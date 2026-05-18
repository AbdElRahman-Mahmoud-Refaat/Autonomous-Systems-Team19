#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class MS5SpeedController(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Speed_Controller_Team_19')
        self.declare_parameter('odom_topic', '/ms5/filtered_odom')
        self.declare_parameter('desired_speed_topic', '/ms4/desired_speed')
        self.declare_parameter('speed_cmd_topic', '/ms3/speed_cmd')
        self.declare_parameter('max_speed_cmd', 0.45)
        self.declare_parameter('finish_x', 9.70)
        self.declare_parameter('odom_timeout', 1.0)
        self.declare_parameter('cmd_timeout', 0.8)
        self.declare_parameter('control_rate', 40.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.desired_speed_topic = str(self.get_parameter('desired_speed_topic').value)
        self.speed_cmd_topic = str(self.get_parameter('speed_cmd_topic').value)
        self.max_speed_cmd = float(self.get_parameter('max_speed_cmd').value)
        self.finish_x = float(self.get_parameter('finish_x').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.target_speed = 0.0
        self.current_speed = 0.0
        self.current_x = 0.0
        self.last_odom_time = None
        self.last_cmd_time = None

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Float64, self.desired_speed_topic, self.desired_speed_callback, 20)
        self.pub = self.create_publisher(Float64, self.speed_cmd_topic, 20)
        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info('MS5 speed controller started using filtered odometry.')

    def desired_speed_callback(self, msg: Float64):
        self.target_speed = max(0.0, min(float(msg.data), self.max_speed_cmd))
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)
        self.last_odom_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        cmd = 0.0
        if self.last_odom_time is not None:
            odom_age = (now - self.last_odom_time).nanoseconds / 1e9
            cmd_age = 0.0 if self.last_cmd_time is None else (now - self.last_cmd_time).nanoseconds / 1e9
            if odom_age <= self.odom_timeout and cmd_age <= self.cmd_timeout and self.current_x < self.finish_x:
                # Pass-through: planner gives desired simulation speed directly.
                cmd = self.target_speed
        self.pub.publish(Float64(data=float(cmd)))
        self.get_logger().info(f'[MS5 Speed] x={self.current_x:.2f}, actual={self.current_speed:.2f}, cmd={cmd:.2f}', throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MS5SpeedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
