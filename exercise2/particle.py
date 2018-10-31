from geometry_msgs.msg import Pose, Point
import numpy as np


class Particle(object):
    def __init__(self, x, y, rot):
        self.position = Point()
        self.position.x = x
        self.position.y = y

	self.rotation = rot

    def _noisy_update(self, current_value, increment, variance):
        return current_value + increment + np.random.normal(loc=0, scale=variance)

    def estimate_update_position(self, prev_tstamp, curr_tstamp, x_vel, y_vel, rot_vel):

	# TODO calculate the new position (in the *_updated variables) based on the supplied timestamps and velocities, given the particles current position (self.position.x, self.position.y, self.rotation)

        x_updated = #TODO
        y_updated = #TODO
        rot_updated = #TODO

        return Particle(x_updated, y_updated, rot_updated)

    def probability_of_pose(self, laser_scan, world_map):
        return None
        # TODO: calculate the probability that, given our current
        # pose estimate on world_map, we would get a laser scan that
        # looks like laser_scan

    def ros_pose(self):
        # Represent my position as ROS Pose Message class
        return Pose(self.position, self.rotation)
