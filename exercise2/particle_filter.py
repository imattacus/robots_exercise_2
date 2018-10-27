#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
import numpy as np

from particle import Particle


def publish_particles():    
    pose_array = PoseArray()
    pose_array.header.frame_id = '1'
    pose_array.header.stamp = rospy.Time.now()

    for particle in current_particles:
        pose_array.poses.append(particle.ros_pose())

    particle_publisher.publish(pose_array)


def move_particles(transform):
    # Given the control signal we sent to the motors, update the
    # positions of all particles according to their motion model
    current_particles = np.fromiter((particle.estimate_update_position(transform)
                                     for particle in current_particles),
                                    current_particles.dtype)
    publish_particles()


def resample_particles(laser_scan):
    # Given a laser scan, resample our particles according to their
    # estimate of the probability of that scan occurring
    weights = np.fromiter((particle.probability_of_pose(laser_scan, world_map)
                           for particle in current_particles),
                          current_particles.dtype)
    current_particles = np.random.choice(current_particles,
                                         size=len(current_particles),
                                         p=weights/weights.sum())
    publish_particles()


if __name__ == '__main__':
    rospy.init_node('stuart_localisation')

    global current_particles = # TODO: initialise random particle distribution (should be numpy array)
    global world_map = # TODO

    rospy.Subscriber('cmd_vel', Twist, move_particles)
    rospy.Subscriber('base_scan', LaserScan, resample_particles)

    global particle_publisher = rospy.Publisher('particle_cloud',
                                                PoseArray,
                                                queue_size=1)

    rospy.spin()
