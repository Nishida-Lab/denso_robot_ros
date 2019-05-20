#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Basic
import sys
import copy
from math import *
from time import time
# ROS
import rospy
import rosparam
# Moveit
import moveit_commander
# == Messages ==
from operation_msgs.msg import PlanningRequest
from geometry_msgs.msg import PoseStamped
# for solve IK
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
# for Start state
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory
# for Planned path
from denso_robot_execute.msg import Plan
from std_msgs.msg import Header


class ParallelPlanner(object):

    def __init__(self):
        # ========= Task queue ======== #
        self.task_q = []

        # ========= Subscriber ======== #
        self.reqest_sub = rospy.Subscriber('/planning_request', PlanningRequest, self.request_callback)

        # ========== Moveit init ========== #
        self.dof = 6
        # moveit_commander init
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        # Set the planning time
        self.planning_limitation_time = 5.0
        self.arm.set_planning_time(self.planning_limitation_time)
        self.start_state = self.robot.get_current_state()
        # for remove mhand2 finger link
        self.start_state.joint_state.name = self.start_state.joint_state.name[:self.dof]
        self.start_state.joint_state.position = self.start_state.joint_state.position[:self.dof]

        # ========== Plan publisher ======== #
        self.plan_pub = rospy.Publisher('/planning_result', Plan, queue_size=6)

        # ========== Solve IK service ======== #
        # rospy.wait_for_service('/compute_ik')
        # try:
        #     self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        # except rospy.ServiceException as e:
        #     rospy.logerr('Activation service failed:{}'.format(e))
        #
        rospy.loginfo("Parallel Planner Initialized")

    # -------- Plannning & Execution -------- #
    def get_plan(self, target):
        # Set argument start state
        self.start_state.joint_state.header.stamp = rospy.Time.now()
        self.arm.set_start_state(self.start_state)

        # Set goal pose
        # ik_req = PositionIKRequest()
        # ik_req.group_name = 'arm'
        # ik_req.robot_state = self.start_state
        # ik_req.avoid_collisions = True
        # ik_req.pose_stamped = target
        #
        # rospy.logerr(ik_req)
        #
        # try:
        #     ret = self.ik_service(ik_req)
        # except rospy.ServiceException as e:
        #     rospy.logerr('compute IK service failed:{}'.format(e))
        #     return False
        #
        # if not ret.solution.joint_state:
        #     rospy.logerr('Could not obtein valid IK solution')
        #     return False
        #
        # ret.solution.joint_state.header.stamp = rospy.Time.now()
        # rospy.logerr(ret)
        retry_count = 0
        retry_limit = 10
        ik_flag = False
        while retry_count < retry_limit:
            try:
                self.arm.set_joint_value_target(target.request)
                ik_flag = True
                break
            except:
                retry_count += 1
                rospy.logwarn("IK failed, retry...")
        if ik_flag is False:
            rospy.logerr("IK failed, abort...")
        # self.arm.set_joint_value_target(ret.solution.joint_state)

        # plan
        plan = RobotTrajectory()
        counter = 0
        start_time = time()
        while len(plan.joint_trajectory.points) == 0 :
            plan = self.arm.plan()
            counter+=1
            self.arm.set_planning_time(self.planning_limitation_time+counter*5.0)
            if counter > 1 :
                return False
        planning_time = time() - start_time
        self.arm.set_planning_time(self.planning_limitation_time)

        rospy.loginfo("!! Got a plan !!")
        # publish the plan
        pub_msg = Plan()
        pub_msg.planning_time = planning_time
        pub_msg.start_state = self.start_state
        pub_msg.start_state.joint_state.header.stamp = rospy.Time.now()
        pub_msg.trajectory = plan
        pub_msg.grasp = target.grasp
        # rospy.logwarn(pub_msg)
        self.plan_pub.publish(pub_msg)
        self.arm.clear_pose_targets()
        # return goal state from generated trajectory
        goal_state = JointState()
        goal_state.header = Header()
        goal_state.header.stamp = rospy.Time.now()
        goal_state.name = plan.joint_trajectory.joint_names[:]
        goal_state.position = plan.joint_trajectory.points[-1].positions[:]
        self.start_state.joint_state = goal_state
        return True

    def is_task(self):
        if self.task_q:
            return True
        else:
            return False

    def request_callback(self, target):
        self.task_q.append(target)

    def planning(self):
        rospy.loginfo("(-O-) Task start (-O-)")
        # initialize
        ret = self.get_plan(self.task_q.pop(0))
        if ret is True:
            rospy.loginfo("(^O^) Task finished (^O^)")
        else:
            rospy.logerr("(xOx) Could not get a valid path (xOx)")


if __name__ == '__main__':
    rospy.init_node("parallel_planner")
    parallel_planner = ParallelPlanner()
    while not rospy.is_shutdown():
        if parallel_planner.is_task():
            parallel_planner.planning()

    rospy.spin()
