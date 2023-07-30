import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose as TurtlePose
from geometry_msgs.msg import Twist, Accel


class Traj1dDifferentiator(Node):
    '''
    TODO
    '''

    def __init__(self):
        super().__init__('traj1d_differentiator')

        # A real implementation should take time into account - ideally by using PoseStamped and TwistStamped.
        # This simple implementation assumes that the pose is published at a fixed rate.
        self.time_delta = 0.1

        self.previous_pose = None
        self.previous_twist = None

        self.twist_pub = self.create_publisher(Twist, 'twist', 10)
        self.accel_pub = self.create_publisher(Accel, 'accel', 10)
        self.pose_sub = self.create_subscription(
            TurtlePose, 'pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        if self.previous_pose is not None:
            twist = Twist()
            twist.linear.x = (msg.x - self.previous_pose.x) / self.time_delta
            twist.linear.y = (msg.y - self.previous_pose.y) / self.time_delta
            twist.angular.z = (
                msg.theta - self.previous_pose.theta) / self.time_delta
            self.twist_pub.publish(twist)
            if self.previous_twist is not None:
                accel = Accel()
                accel.linear.x = (
                    twist.linear.x - self.previous_twist.linear.x) / self.time_delta
                accel.linear.y = (
                    twist.linear.y - self.previous_twist.linear.y) / self.time_delta
                accel.angular.z = (
                    twist.angular.z - self.previous_twist.angular.z) / self.time_delta
                self.accel_pub.publish(accel)
            self.previous_twist = twist
        self.previous_pose = msg


def main(args=None):
    rclpy.init(args=args)

    traj1d_differentiator = Traj1dDifferentiator()

    rclpy.spin(traj1d_differentiator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
