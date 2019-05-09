#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Basic
import sys
import copy
from math import *
# ROS
import rospy
# == Action lib client ==
import actionlib
from denso_robot_execute.msg import ExecutePlanAction
from denso_robot_execute.msg import ExecutePlanActionGoal
# == Messages ==
# for execution
from denso_robot_execute.msg import Plan
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class ParallelExecuter(object):

    def __init__(self):
        # ========== Action lib client init ========== #
        self.client = actionlib.SimpleActionClient('execute_action', ExecutePlanAction)
        self.client.wait_for_server()

        # ======== Subscriber ======== #
        self.plan_sub = rospy.Subscriber('/planning_result', Plan, self.plan_callback)

        # ======== Publisher ======== #
        self.display_hp_pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=6)

        # Execution Speed
        self.exe_speed_rate = rospy.get_param('~exe_speed_rate', 1.0)
        print self.exe_speed_rate

        # task queue
        self.task_q = []

        rospy.loginfo("Parallel Executer Initialized")

    # -------- Plannning & Execution -------- #
    def plan_callback(self, plan):
        rospy.loginfo('receive plan')
        self.task_q.append(plan)

    def execute(self):
        rospy.loginfo("Start Task")
        # Get latest task plan
        plan = self.task_q[0]
        for points in plan.trajectory.joint_trajectory.points:
            tfs = points.time_from_start.to_sec()
            tfs /= self.exe_speed_rate
            points.time_from_start = rospy.Duration(tfs)

        # Display the Trajectory
        start_state = JointState()
        start_state.header = Header()
        start_state.header.stamp = rospy.Time.now()
        start_state.name =  plan.trajectory.joint_trajectory.joint_names[:]
        start_state.position = plan.trajectory.joint_trajectory.points[-1].positions[:]
        moveit_start_state = RobotState()
        moveit_start_state.joint_state = start_state
        pub_display_msg = DisplayTrajectory()
        pub_display_msg.model_id = "vs087"
        pub_display_msg.trajectory.append(plan.trajectory)
        pub_display_msg.trajectory_start = moveit_start_state
        self.display_hp_pub.publish(pub_display_msg)

        # Send Action and Wait result
        goal = ExecutePlanAction().action_goal.goal
        goal.planning_time = plan.planning_time
        goal.start_state = plan.start_state
        # goal.start_state.joint_state.header.stamp = rospy.Time.now()
        goal.trajectory = plan.trajectory
        # rospy.logwarn(goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # Update the task queue
        self.task_q.pop(0)
        rospy.loginfo("End Task")

    def isTask(self):
        if not self.task_q:
            return False
        return True

    def shutdown(self):
        rospy.logwarn("(xOx) Aborted (xOx)")


if __name__ == '__main__':
    rospy.init_node("handring_parallel_executor")
    parallel_executer = ParallelExecuter()
    while not rospy.is_shutdown():
        if parallel_executer.isTask():
            parallel_executer.execute()

    rospy.on_shutdown(parallel_executer.shutdown)
