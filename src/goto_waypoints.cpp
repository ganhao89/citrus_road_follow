#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoToGoal
{ 


public:
    GoToGoal():
    ac("move_base", true)
    {
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
	goal_sub = n_.subscribe("/way_points", 1, &GoToGoal::waypointsCallback,this);
    }

    ~GoToGoal()
    {
    }
    void waypointsCallback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
    {
      //MoveBaseClient ac("move_base", true);
      //while(!ac.waitForServer(ros::Duration(5.0))){
      //      ROS_INFO("Waiting for the move_base action server to come up");
      //}
      goal_.target_pose = msg->target_pose;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal_);

      ac.waitForResult(ros::Duration(10.0));

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved to the waypoint");
      else
        ROS_INFO("The base failed to move to the waypoint for some reason");
    }
private:
    ros::NodeHandle n_;
    move_base_msgs::MoveBaseGoal goal_;
    ros::Subscriber goal_sub;
    MoveBaseClient ac;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_goals");
  GoToGoal goalobj;
  ros::spin();
  return 0;
}


