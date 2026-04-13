import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import time


class OLRNode(Node):

    def __init__(self):
        super().__init__('OLR_node')

        # Parameters
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.start_time = time.time()

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

        self.timer = self.create_timer(0.1, self.publish_command)

        self.get_logger().info("OLR Node Started (3-second sequence mode)")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'linear_speed':
                self.linear_speed = float(param.value)
            elif param.name == 'angular_speed':
                self.angular_speed = float(param.value)

        return SetParametersResult(successful=True)

    def publish_command(self):
        msg = Twist()
        elapsed = time.time() - self.start_time

        # Logical motion sequence
        if elapsed < 5:
            # Forward
            lin = -self.linear_speed
            ang = 0.0
            phase = "FORWARD"

        elif elapsed < 10:
            # Backward
            lin = self.linear_speed
            ang = 0.0
            phase = "BACKWARD"

        elif elapsed < 15:
            # Forward Left
            lin = - self.linear_speed
            ang = self.angular_speed
            phase = "FORWARD LEFT"

        elif elapsed < 20:
            # Forward Right
            lin = -self.linear_speed
            ang = -self.angular_speed
            phase = "FORWARD RIGHT"

        else:
            # Stop
            lin = 0.0
            ang = 0.0
            phase = "STOP"

        # Gazebo model fix:
        # negative linear.x in Gazebo = forward motion for this model
        msg.linear.x = -float(lin)
        msg.angular.z = float(ang)

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"{phase} | logical_input=({lin:.2f}, {ang:.2f}) | "
            f"sent_to_gazebo=({msg.linear.x:.2f}, {msg.angular.z:.2f})"
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        self.get_logger().info(
            f"Position: ({x:.2f}, {y:.2f}) | "
            f"Velocity: {vx:.2f} | Angular: {wz:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OLRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
