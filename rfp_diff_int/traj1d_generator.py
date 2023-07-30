import numpy as np

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose as TurtlePose


class Traj1dGenerator(Node):
    '''
    Publish a 1D trajectory as turtlesim/Pose.
    Goes along well with turtlesim, turtle_teleporter and traj1d_differentiator.
    '''

    def __init__(self):
        super().__init__('traj1d_generator')
        self.times = np.arange(0, 151)
        self.accelerations = np.array([0]
                                      + [0.15] * 15
                                      + [0] * 29
                                      + np.linspace(0.0, -0.2, 11).tolist()
                                      + [0] * 19
                                      + np.linspace(0.0, 0.2, 11).tolist()
                                      + [0] * 4
                                      + np.linspace(0.0, -0.3, 6).tolist()
                                      + [0] * 14
                                      + np.linspace(0.0, (0.3*6) /
                                                    11, 11).tolist()
                                      + [0] * 20
                                      + [-0.225] * 10)
        self.velocities = np.cumsum(self.accelerations)
        self.positions = np.cumsum(self.velocities)

        self.traj_index = 0

        self.pose_pub = self.create_publisher(TurtlePose, 'cmd_pose', 10)
        self.timer_period = 0.1  # [seconds]; runs trajectory at 10x speed
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        pose = TurtlePose()
        pose.x = self.positions[self.traj_index]
        self.get_logger().info(f't={self.times[self.traj_index]}: x={pose.x}')
        self.pose_pub.publish(pose)
        self.traj_index += 1
        if self.traj_index == len(self.times):
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    traj1d_generator = Traj1dGenerator()

    try:
        rclpy.spin(traj1d_generator)
    except SystemExit:
        rclpy.logging.get_logger('rclpy').info(
            'Trajectory finished. Shutting down.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
