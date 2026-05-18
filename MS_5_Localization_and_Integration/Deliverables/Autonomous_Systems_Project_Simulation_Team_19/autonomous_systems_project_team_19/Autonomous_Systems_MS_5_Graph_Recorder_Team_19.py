#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from .utils import yaw_from_quaternion


class MS5GraphRecorder(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Graph_Recorder_Team_19')
        self.declare_parameter('track_id', 2)
        self.declare_parameter('ideal_odom_topic', '/model/vehicle_blue/odometry')
        self.declare_parameter('noisy_odom_topic', '/ms5/noisy_odom')
        self.declare_parameter('filtered_odom_topic', '/ms5/filtered_odom')
        self.declare_parameter('desired_speed_topic', '/ms4/desired_speed')
        self.declare_parameter('desired_lane_topic', '/ms4/desired_lane_y')
        self.declare_parameter('steering_cmd_topic', '/ms3/steering_cmd')
        self.declare_parameter('speed_cmd_topic', '/ms3/speed_cmd')
        self.declare_parameter('sample_rate', 20.0)

        self.track_id = int(self.get_parameter('track_id').value)
        self.folder = os.path.expanduser(f'~/ros2_ws/graphs_track{self.track_id}')
        os.makedirs(self.folder, exist_ok=True)
        self.csv_path = os.path.join(self.folder, f'Autonomous_Systems_Project_Simulation_MS_5_Track_{self.track_id}_Log_Team_19.csv')

        self.data = {
            'ideal_x': math.nan, 'ideal_y': math.nan, 'ideal_theta': math.nan, 'ideal_speed': math.nan,
            'noisy_x': math.nan, 'noisy_y': math.nan, 'noisy_theta': math.nan, 'noisy_speed': math.nan,
            'filtered_x': math.nan, 'filtered_y': math.nan, 'filtered_theta': math.nan, 'filtered_speed': math.nan,
            'desired_speed': math.nan, 'desired_y': math.nan, 'steering_cmd': math.nan, 'speed_cmd': math.nan,
        }
        self.start_time = self.get_clock().now()
        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.header = ['time'] + list(self.data.keys())
        self.writer.writerow(self.header)

        self.create_subscription(Odometry, str(self.get_parameter('ideal_odom_topic').value), self.ideal_cb, 20)
        self.create_subscription(Odometry, str(self.get_parameter('noisy_odom_topic').value), self.noisy_cb, 20)
        self.create_subscription(Odometry, str(self.get_parameter('filtered_odom_topic').value), self.filtered_cb, 20)
        self.create_subscription(Float64, str(self.get_parameter('desired_speed_topic').value), lambda m: self.set_value('desired_speed', m.data), 20)
        self.create_subscription(Float64, str(self.get_parameter('desired_lane_topic').value), lambda m: self.set_value('desired_y', m.data), 20)
        self.create_subscription(Float64, str(self.get_parameter('steering_cmd_topic').value), lambda m: self.set_value('steering_cmd', m.data), 20)
        self.create_subscription(Float64, str(self.get_parameter('speed_cmd_topic').value), lambda m: self.set_value('speed_cmd', m.data), 20)

        rate = float(self.get_parameter('sample_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.record_row)
        self.get_logger().info(f'MS5 graph recorder saving to: {self.csv_path}')

    def set_value(self, key, value):
        self.data[key] = float(value)

    def odom_values(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        return (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
            math.sqrt(vx * vx + vy * vy),
        )

    def ideal_cb(self, msg):
        x, y, th, v = self.odom_values(msg)
        self.data.update({'ideal_x': x, 'ideal_y': y, 'ideal_theta': th, 'ideal_speed': v})

    def noisy_cb(self, msg):
        x, y, th, v = self.odom_values(msg)
        self.data.update({'noisy_x': x, 'noisy_y': y, 'noisy_theta': th, 'noisy_speed': v})

    def filtered_cb(self, msg):
        x, y, th, v = self.odom_values(msg)
        self.data.update({'filtered_x': x, 'filtered_y': y, 'filtered_theta': th, 'filtered_speed': v})

    def record_row(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.writer.writerow([t] + [self.data[k] for k in self.data.keys()])
        self.file.flush()

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MS5GraphRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
