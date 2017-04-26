#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//tell the action client that we want to spin a thread by default
MoveBaseClient ac("move_base", true);

void waypointsCallback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = msg->target_pose;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult(ros::Duration(1.0));

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_goals");
  ros::NodeHandle n; 
   //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Subscriber goal_sub = n.subscribe("way_points", 1, waypointsCallback);
  ros::spin();
  return 0;
}



