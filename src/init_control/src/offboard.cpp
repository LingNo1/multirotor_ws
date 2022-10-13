#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "geometry_msgs/PoseStamped.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

float global_height = 0.0;
//高度
void get_height(const sensor_msgs::Range::ConstPtr& height)
{
    global_height = height->range;
    // ROS_INFO("height:%.2f",global_height);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //初始化
    ros::init(argc,argv,"my_takeoff");
    ros::NodeHandle nh;

    //高度订阅
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>
        ("/mavros/distance_sensor/hrlv_ez4_pub",10,get_height);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    //模式切换
    ros::Publisher multi_cmd_vel_flu_pub = nh.advertise<geometry_msgs::Twist>
        ("/cmd_vel_flu",1);
    ros::Publisher multi_cmd_pub = nh.advertise<std_msgs::String>
        ("/cmd",3);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    std_msgs::String cmd;
    geometry_msgs::Twist twist;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.8;


    ros::Rate rate(20);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    cmd.data = "OFFBOARD";
    multi_cmd_pub.publish(cmd);

    while(ros::ok())
    {
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed");

            last_request = ros::Time::now();
        }
        if ( current_state.armed && global_height > 0.70 && (ros::Time::now() - last_request > ros::Duration(10.0)))
        {
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = 0.0;

            for(int i = 100; ros::ok() && i > 0; --i)
            {
                multi_cmd_vel_flu_pub.publish(twist);
                // local_pos_pub.publish(pose);

                cmd.data = "HOVER";
                multi_cmd_pub.publish(cmd);

                ros::spinOnce();
                rate.sleep();
            }
            last_request = ros::Time::now();
            break;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();

        rate.sleep();
    }
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        ros::Duration(3.0).sleep();
        cmd.data = "AUTO.LAND";
        multi_cmd_pub.publish(cmd);
    }
    return 0;
}