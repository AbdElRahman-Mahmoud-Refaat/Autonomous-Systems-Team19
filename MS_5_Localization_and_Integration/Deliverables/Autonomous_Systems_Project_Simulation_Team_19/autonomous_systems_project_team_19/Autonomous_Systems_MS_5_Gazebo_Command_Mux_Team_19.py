#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from .utils import clamp


class MS5GazeboCommandMux(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Gazebo_Command_Mux_Team_19')
        self.declare_parameter('speed_cmd_topic', '/ms3/speed_cmd')
        self.declare_parameter('steering_cmd_topic', '/ms3/steering_cmd')
        self.declare_parameter('cmd_vel_topic', '/model/vehicle_blue/cmd_vel')
        self.declare_parameter('wheel_base', 0.22)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_steering_angle', 0.24)
        self.declare_parameter('max_yaw_rate', 0.90)
        self.declare_parameter('publish_rate', 40.0)
        self.declare_parameter('command_timeout', 1.0)

        self.speed_cmd_topic = str(self.get_parameter('speed_cmd_topic').value)
        self.steering_cmd_topic = str(self.get_parameter('steering_cmd_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_steering = float(self.get_parameter('max_steering_angle').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.timeout = float(self.get_parameter('command_timeout').value)

        self.latest_speed = 0.0
        self.latest_steering = 0.0
        self.last_speed_time = None
        self.last_steer_time = None

        self.create_subscription(Float64, self.speed_cmd_topic, self.speed_callback, 20)
        self.create_subscription(Float64, self.steering_cmd_topic, self.steering_callback, 20)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)
        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)
        self.get_logger().info('MS5 Gazebo command mux started. Converts speed + steering to /cmd_vel.')

    def speed_callback(self, msg: Float64):
        self.latest_speed = float(msg.data)
        self.last_speed_time = self.get_clock().now()

    def steering_callback(self, msg: Float64):
        self.latest_steering = float(msg.data)
        self.last_steer_time = self.get_clock().now()

    def is_old(self, t):
        if t is None:
            return True
        return (self.get_clock().now() - t).nanoseconds / 1e9 > self.timeout

    def publish_cmd(self):
        v = 0.0 if self.is_old(self.last_speed_time) else clamp(self.latest_speed, 0.0, self.max_linear_speed)
        delta = 0.0 if self.is_old(self.last_steer_time) else clamp(self.latest_steering, -self.max_steering, self.max_steering)
        omega = 0.0
        if abs(self.wheel_base) > 1e-6:
            omega = v * math.tan(delta) / self.wheel_base
        omega = clamp(omega, -self.max_yaw_rate, self.max_yaw_rate)

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MS5GazeboCommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
