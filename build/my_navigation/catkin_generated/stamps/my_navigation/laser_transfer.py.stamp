#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import Range

rospy.init_node('mavros_laser_transfer')
pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)
tfBuffer = Buffer()
tflistener = TransformListener(tfBuffer)
local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
rate = rospy.Rate(30)

def cartographer(Range):
    global local_pose, height
    tfstamped = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    local_pose.header.stamp = rospy.Time().now()
    local_pose.pose.position = tfstamped.transform.translation
    local_pose.pose.position.z = Range.range
    local_pose.pose.orientation = tfstamped.transform.rotation
    pose_pub.publish(local_pose)
    rate.sleep()
    
if __name__ == '__main__':
    height_sub = rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub",Range,cartographer, queue_size=10)
    rospy.spin()