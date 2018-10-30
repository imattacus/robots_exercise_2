#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from random import Random

from particle import Particle
from map import Map


def publish_particles():
    # Create and publish PoseArray for rviz
    pose_array = PoseArray()
    pose_array.header.frame_id = '1'  # TODO: Should this be 'map'?
    pose_array.header.stamp = rospy.Time.now()

    for particle in particles[iteration, :]:
        pose_array.poses.append(particle.ros_pose())

    particle_publisher.publish(pose_array)


def move_particles(transform):
    # Given the control signal we sent to the motors, update the
    # positions of all particles according to their motion model
    particles[iteration, :] = np.fromiter((particle.estimate_update_position(transform)
                                           for particle in particles[iteration - 1, :]),
                                          particles[iteration - 1, :].dtype)
    publish_particles()


def resample_particles(laser_scan):
    # Given a laser scan, resample our particles according to their
    # estimate of the probability of that scan occurring
    iteration += 1

    weights = np.fromiter((particle.probability_of_pose(laser_scan, world_map)
                           for particle in particles[iteration, :]),
                          particles[iteration, :].dtype)
    particles[iteration, :] = np.random.choice(particles[iteration - 1, :],
                                               size=particles[iteration - 1, :].size,
                                               p=weights / weights.sum())
    publish_particles()


def fresh_particles():
    particles[iteration, :] = [random_particle() for a in range(0, particle_count)]
    iteration += 1


def random_particle():
    valid_pose = False
    while not valid_pose:
        x = Random.randint(0, world_map.width)
        y = Random.randint(0, world_map.height)
        valid_pose = world_map.valid_pos(x, y)
    angle = Random.random() * 360  # TODO: Deg or Rad??
    return Particle(x, y, angle)


if __name__ == '__main__':
    rospy.init_node('stuart_localisation')

    global particles
    global iteration
    global world_map
    global particle_count

    particle_count = 5
    particles = np.array()
    iteration = 0
    world_map = Map(None)

    fresh_particles()

    rospy.Subscriber('cmd_vel', Twist, move_particles)
    rospy.Subscriber('base_scan', LaserScan, resample_particles)

    particle_publisher = rospy.Publisher('particle_cloud',
                                         PoseArray,
                                         queue_size=1)

    rospy.spin()
