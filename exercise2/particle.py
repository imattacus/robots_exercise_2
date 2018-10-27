from geometry_msgs.msg import Pose, Point, Quaternion

class Particle(object):
    def __init__(self, x, y, rotation):
        self.position = Point()
        self.position.x = x
        self.position.y = y

        self.rotation = Quaternion()
        self.rotation.x = rotation

    def estimate_update_position(self, transform):
        # TODO: generate a new Particle, with position given by odometry
        # model of type Twist (and a little bit of noise)

    def probability_of_pose(self, laser_scan, world_map):
        # TODO: calculate the probability that, given our current
        # pose estimate on world_map, we would get a laser scan that
        # looks like laser_scan

    def ros_pose(self):
        # Represent my position as ROS Pose Message class
        return Pose(self.position, self.rotation)
