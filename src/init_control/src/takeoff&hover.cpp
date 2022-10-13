#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

float global_height = 0.0;

//高度
void get_height(const sensor_msgs::Range::ConstPtr& height)
{
    global_height = height->range;
    ROS_INFO("现在高度为：%.2f米",global_height);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //初始化
    ros::init(argc,argv,"my_takeoff");
    ros::NodeHandle nh;

    //高度订阅
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub",10,get_height);
    
    //模式切换
    ros::Publisher multi_cmd_vel_flu_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_flu",1);
    ros::Publisher multi_cmd_pub = nh.advertise<std_msgs::String>("/cmd",3);

    float forward  = 0.0 ,leftward  = 0.0,upward  = 0.0,angular = 0.0;
    std_msgs::String cmd;
    geometry_msgs::Twist twist;

    ros::Duration(3.0).sleep();
    cmd.data = "OFFBOARD";
    multi_cmd_pub.publish(cmd);

    ros::Duration(1.0).sleep();
    cmd.data = "ARM";
    twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward + atof(argv[1]);
    twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular;
    multi_cmd_vel_flu_pub.publish(twist);
    multi_cmd_pub.publish(cmd);
    ROS_INFO("起飞!");
    ros::Duration(1.0).sleep();


    ros::Rate r(0.1);
    while(ros::ok())
    {
        ros::spinOnce();
        if(global_height < 1.5)
            ROS_INFO("高度还不够!",global_height);
        else
        {
            ROS_INFO("高度够了!");
            twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward;
            twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular;
            multi_cmd_vel_flu_pub.publish(twist);
            cmd.data = "HOVER";
            multi_cmd_pub.publish(cmd);
            ROS_INFO("悬停！");
            break;
        }
        r.sleep();
    }

    cmd.data = "OFFBOARD";
    multi_cmd_pub.publish(cmd);
    ros::Duration(3.0).sleep();

    return 0;
}
