#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char* argv[]){
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  int point_num = atoi(argv[1]);
  for (int i = 1; i <= point_num; i++)
  {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = atoi(argv[i+1]);
    goal.target_pose.pose.position.y = atoi(argv[i+1+point_num]);
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("到达Map坐标系下的(%d,%d)坐标点",atoi(argv[i+1]),atoi(argv[i+1+point_num]));
    else
      ROS_INFO("The base failed to move for some reason");
    
    ROS_WARN("开始任务,执行时间为1秒!");
    ros::Duration(1).sleep();
    ROS_INFO("该任务执行完毕，开始下一任务!");
  }

  goal.target_pose.pose.position.x = 0.2;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("到达Map坐标系下的(0.5,0)坐标点");
  else
    ROS_INFO("The base failed to move for some reason");
  
  std_msgs::String cmd ;
  cmd.data = "AUTO.LAND";
  geometry_msgs::Twist twist;

  ros::Publisher multi_cmd_vel_pub= nh.advertise<geometry_msgs::Twist>("/xtdrone/iris_0/cmd_vel_flu",1);
  ros::Publisher multi_cmd_pub = nh.advertise<std_msgs::String>("/xtdrone/iris_0/cmd",3);
  ros::Duration(1).sleep();

  multi_cmd_pub.publish(cmd);

  return 0;
}