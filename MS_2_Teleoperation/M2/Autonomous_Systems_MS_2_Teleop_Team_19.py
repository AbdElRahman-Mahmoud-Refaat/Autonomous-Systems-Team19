import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TeleopNode(Node):

    def __init__(self):
        super().__init__('Autonomous_Systems_MS_2_Teleop_Team_19')

        self.publisher_ = self.create_publisher(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.odom_callback,
            10
        )

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.linear_step = 0.2
        self.angular_step = 0.2

        self.settings = termios.tcgetattr(sys.stdin)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Teleop Node Started")
        self.get_logger().info("Use arrow keys:")
        self.get_logger().info("UP    -> increase forward speed")
        self.get_logger().info("DOWN  -> increase backward speed")
        self.get_logger().info("LEFT  -> steer left")
        self.get_logger().info("RIGHT -> steer right")
        self.get_logger().info("SPACE -> stop vehicle")
        self.get_logger().info("Press q to quit")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        key = ''
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def control_loop(self):
        key = self.get_key()

        if key == '\x1b[A':          # UP
            self.linear_speed += self.linear_step
        elif key == '\x1b[B':        # DOWN
            self.linear_speed -= self.linear_step
        elif key == '\x1b[D':        # LEFT
            self.angular_speed += self.angular_step
        elif key == '\x1b[C':        # RIGHT
            self.angular_speed -= self.angular_step
        elif key == ' ':             # SPACE
            self.linear_speed = 0.0
            self.angular_speed = 0.0
        elif key == 'q':
            self.get_logger().info("Quitting Teleop Node")
            rclpy.shutdown()
            return

        msg = Twist()
        msg.linear.x = float(self.linear_speed)
        msg.angular.z = float(self.angular_speed)
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Command -> linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}"
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        self.get_logger().info(
            f"State -> Position: ({x:.2f}, {y:.2f}) | "
            f"Velocity: {vx:.2f} | "
            f"Angular: {wz:.2f} | "
            f"Orientation(z,w): ({orientation_z:.2f}, {orientation_w:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
