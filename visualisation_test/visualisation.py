#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def callback( sensor_data ):
    base_data = Twist()
    base_data.angular.z = 0.5
    vel_pub.publish( base_data )


def odom_cb(msg):
    print(msg.pose.pose)
    pose_data = PoseArray()
    poses = {msg.pose.pose}
    pose_data.header.frame_id = "map"
    pose_data.header.stamp = rospy.Time.now()
    pose_data.poses = poses
    pose_pub.publish(pose_data)

if __name__ == '__main__':
    print("Running node")
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.Subscriber('base_scan', LaserScan, callback)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    pose_pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=100)
    rospy.spin()
