#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def callback(f, sensor_data):
    f.write(str(sensor_data))

if __name__ == "__main__":
    with open("/data/private/robot/laser_test.txt", "w") as f:
        rospy.init_node("laser_test")
        rospy.Subscriber("base_scan", LaserScan, lambda d: callback(f, d))
        rospy.spin()
