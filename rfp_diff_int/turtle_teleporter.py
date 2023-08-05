import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import TeleportAbsolute


class TurtleTeleporter(Node):
    '''
    Subscribes to a turtlesim/Pose topic and teleports turtesim turtles there
    '''

    def __init__(self):
        super().__init__('turtle_teleporter')
        self.teleport_service = self.create_client(
            TeleportAbsolute, 'teleport_absolute')
        self.pose_sub = self.create_subscription(
            TurtlePose, 'cmd_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        turtlesim_start_pos = 5.544445
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = turtlesim_start_pos + msg.x / 50.
        teleport_request.y = turtlesim_start_pos  # starting position
        teleport_request.theta = msg.theta
        self.get_logger().info(
            f'Teleporting turtle to x={msg.x}, y={msg.y}, theta={msg.theta} '
            '(mapped to x={teleport_request.x}, y={teleport_request.y}, '
            'theta={teleport_request.theta})')
        self.teleport_service.call_async(teleport_request)


def main(args=None):
    rclpy.init(args=args)

    turtle_teleporter = TurtleTeleporter()

    rclpy.spin(turtle_teleporter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
