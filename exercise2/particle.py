from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np


class Particle(object):
    def __init__(self, x, y, rotation):
        self.position = Point()
        self.position.x = x
        self.position.y = y

        self.rotation = Quaternion()
        self.rotation.x = rotation

    def _noisy_update(self, current_value, increment, variance):
        return current_value + increment + np.random.normal(loc=0, scale=variance)

    def estimate_update_position(self, transform):
        # TODO: work out best values for variance
        x_variance = 1
        y_variance = 1
        rot_variance = 1

        x_updated = self._noisy_update(current_value=self.position.x,
                                       increment=transform.linear.x,
                                       variance=x_variance)
        y_updated = self._noisy_update(current_value=self.position.y,
                                       increment=transform.linear.y,
                                       variance=y_variance)
        rot_updated = self._noisy_update(current_value=self.rotatation.x,
                                         increment=transform.angular.z,
                                         variance=rot_variance)

        return Particle(x_updated, y_updated, rot_updated)

    def probability_of_pose(self, laser_scan, world_map):
        # TODO: calculate the probability that, given our current
        # pose estimate on world_map, we would get a laser scan that
        # looks like laser_scan

    def ros_pose(self):
        # Represent my position as ROS Pose Message class
        return Pose(self.position, self.rotation)
