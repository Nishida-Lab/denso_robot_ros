#include <ros/ros.h>
#include <ros/time.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <denso_robot_execute/ExecutePlanAction.h>
#include <actionlib/server/simple_action_server.h>

class ExecutePlanActionServer
{
protected:
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<denso_robot_execute::ExecutePlanAction> as_;
  std::string action_name_;
  denso_robot_execute::ExecutePlanResult result_;
  denso_robot_execute::ExecutePlanGoalConstPtr goal_;
  moveit::planning_interface::MoveGroupInterface group_;

public:

  ExecutePlanActionServer(std::string name, std::string movegroup_name) :
    as_(nh_, name, false),
    action_name_(name),
    group_(movegroup_name)
  {
    as_.registerGoalCallback(boost::bind(&ExecutePlanActionServer::goalCB, this));
    as_.start();
  }

  void goalCB()
  {
    ROS_INFO("Goal Recieived");

    goal_ = as_.acceptNewGoal();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.planning_time_ = goal_->planning_time;
    plan.start_state_ = goal_->start_state;
    plan.trajectory_ = goal_->trajectory;

    group_.setStartState(plan.start_state_);

    // Send the Trajectory & Wait the Execution
    ROS_INFO("Moving...");
    if(group_.execute(plan))
    {
      result_.success = true;
    }
    else
    {
      result_.success = false;
    }
    as_.setSucceeded(result_);
    ROS_INFO("Task Done !!");
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "execute_action_server");

  ExecutePlanActionServer server("execute_action", "arm");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
