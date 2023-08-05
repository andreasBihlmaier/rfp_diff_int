import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist


def wrap_orientation(orientation):
    '''
    Wraps an orientation value to the range [-pi, pi]
    '''
    while orientation > math.pi:
        orientation -= 2 * math.pi
    while orientation < -math.pi:
        orientation += 2 * math.pi
    return orientation


class TwistIntegrator(Node):
    '''
    Integrates a geometry_msgs/Twist topic and publishes the result as a geometry_msgs/Pose
    '''

    def __init__(self):
        super().__init__('twist_integrator')

        # Assumption: Starting pose is all zeros.
        # A real implementation should take the starting pose from a parameter.
        self.current_pose = Pose()

        self.last_time = None

        self.pose_pub = self.create_publisher(Pose, 'calculated_pose', 10)
        self.twist_sub = self.create_subscription(
            Twist, 'twist', self.twist_callback, 10)

    def twist_callback(self, msg):
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        # Assumption: Twist is constant between messages.
        now = self.get_clock().now()
        time_delta = (now - self.last_time).nanoseconds / 1e9

        self.current_pose.position.x += msg.linear.x * time_delta
        self.current_pose.position.y += msg.linear.y * time_delta
        self.current_pose.position.z += msg.linear.z * time_delta
        self.current_pose.orientation.x = wrap_orientation(
            self.current_pose.orientation.x + msg.angular.x * time_delta)
        self.current_pose.orientation.y = wrap_orientation(
            self.current_pose.orientation.y + msg.angular.y * time_delta)
        self.current_pose.orientation.z = wrap_orientation(
            self.current_pose.orientation.z + msg.angular.z * time_delta)
        self.pose_pub.publish(self.current_pose)
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)

    twist_integrator = TwistIntegrator()

    rclpy.spin(twist_integrator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
