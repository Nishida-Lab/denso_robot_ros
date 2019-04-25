#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class ExecuteActionServer
{
protected:
  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;
  control_msgs::FollowJointTrajectoryResult result_;
  trajectory_msgs::JointTrajectory goal_;

public:

  ExecuteActionServer(std::string name) :
    as_(nh_, name, false),
    action_name_(name)
  {
    as_.registerGoalCallback(boost::bind(&ExecuteActionServer::goalCB, this));

    as_.start();
  }

  void goalCB()
  {
    ROS_INFO("Goal Recieived");
    // Change first trajectory position to the current position

    goal_ = as_.acceptNewGoal()->trajectory;

    // Send the Trajectory & Wait the Execution
    ROS_INFO("Moving...");

    result_.error_code = result_.SUCCESSFUL;
    as_.setSucceeded(result_);
    ROS_INFO("Task Done !!");
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "execute_action_server");

  ExecuteActionServer server(ros::this_node::getName());
  ros::spin();

  return 0;
}
