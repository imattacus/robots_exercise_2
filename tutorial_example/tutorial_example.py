#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback( sensor_data ):
    base_data = Twist()
    base_data.angular.z = 0.5
    pub.publish( base_data )

if __name__ == '__main__':
    print("Running node")
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()
