#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import sys

rospy.init_node('mavros_laser_transfer')
pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)
tfBuffer = Buffer()
tflistener = TransformListener(tfBuffer)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
hector = PoseStamped()
height = 0
        
def cartographer():
    global local_pose, height
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            tfstamped = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        except:
            continue
        local_pose.header.stamp = rospy.Time().now()
        local_pose.pose.position = tfstamped.transform.translation
        local_pose.pose.position.z = height
        local_pose.pose.orientation = tfstamped.transform.rotation
        pose_pub.publish(local_pose)
        rate.sleep()
    
if __name__ == '__main__':
	cartographer()