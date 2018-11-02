#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import TwistWithCovariance, Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from random import Random

from particle import Particle
from map import Map

global particles
global prev_tstamp

def publish_particles():
    # Create and publish PoseArray for rviz
    pose_array = PoseArray()
    pose_array.header.frame_id = 'map'
    pose_array.header.stamp = rospy.Time.now()

    for particle in particles:
        pose_array.poses.append(particle.ros_pose())

    particle_publisher.publish(pose_array)


def move_particles(msg):
    global particles
    global prev_tstamp
	
    curr_tstamp = msg.header.stamp
    x_vel = msg.twist.twist.linear.x
    y_vel = msg.twist.twist.linear.y
    rot_vel = msg.twist.twist.angular.z

    # Given the control signal we sent to the motors, update the
    # positions of all particles according to their motion model
    new_particles = np.fromiter((particle.estimate_update_position(prev_tstamp, curr_tstamp, x_vel, y_vel, rot_vel) for particle in particles), particles.dtype)

    particles = new_particles
    prev_tstamp = curr_tstamp
    publish_particles()


def resample_particles(laser_scan):
    global particles
    global world_map
    # Given a laser scan, resample our particles according to their
    # estimate of the probability of that scan occurring

    weights = np.fromiter((particle.probability_of_pose(laser_scan, world_map)
                           for particle in particles),
                          particles.dtype)
    particles = np.random.choice(particles,
                                 size=particles.size,
                                 p=weights / weights.sum())
    publish_particles()


def fresh_particles():
    global particles
    particles = np.array([random_particle() for a in range(0, particle_count)])


def random_particle():
    valid_pose = False
    while not valid_pose:
        x = np.random.randint(0, world_map.width)
        y = np.random.randint(0, world_map.height)
        valid_pose = world_map.valid_pos(x, y)
    angle = np.random.random() * 360  # TODO: Deg or Rad??
    return Particle(x, y, angle)


if __name__ == '__main__':
    rospy.init_node('stuart_localisation')

    global particles
    global iteration
    global world_map
    global particle_count
	global prev_tstamp

    particle_count = 5
    particles = None
    iteration = 0
    world_map = Map("~/catkin_ws/src/robots_exercise_2/stage/map.png")
    prev_tstamp = rospy.Time.now()

    fresh_particles()

    rospy.Subscriber('odom', Odometry, move_particles)
    rospy.Subscriber('base_scan', LaserScan, resample_particles)

    particle_publisher = rospy.Publisher('particle_cloud',
                                         PoseArray,
                                         queue_size=1)

    rospy.spin()
