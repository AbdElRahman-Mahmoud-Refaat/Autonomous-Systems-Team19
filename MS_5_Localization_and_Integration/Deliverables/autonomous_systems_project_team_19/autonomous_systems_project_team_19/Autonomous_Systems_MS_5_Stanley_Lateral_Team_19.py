#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from .utils import yaw_from_quaternion, normalize_angle, clamp


class MS5StanleyLateral(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Stanley_Lateral_Team_19')
        self.declare_parameter('track_id', 2)
        self.declare_parameter('odom_topic', '/ms5/filtered_odom')
        self.declare_parameter('desired_lane_topic', '/ms4/desired_lane_y')
        self.declare_parameter('desired_yaw_topic', '/ms4/desired_yaw')
        self.declare_parameter('desired_cte_topic', '/ms4/desired_cte')
        self.declare_parameter('steering_lock_topic', '/ms4/steering_lock')
        self.declare_parameter('steering_cmd_topic', '/ms3/steering_cmd')
        self.declare_parameter('stanley_gain', 0.80)
        self.declare_parameter('city_stanley_gain', 0.70)
        self.declare_parameter('softening_gain', 0.45)
        self.declare_parameter('heading_gain', 1.6)
        self.declare_parameter('city_heading_gain', 1.00)
        self.declare_parameter('max_steering_angle', 0.46)
        self.declare_parameter('minimum_speed_for_stanley', 0.05)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('steering_filter_alpha', 0.45)
        self.declare_parameter('max_steering_rate', 0.90)
        self.declare_parameter('control_rate', 40.0)
        self.declare_parameter('odom_timeout', 1.0)
        # Track 3 safety/debug requirement: absolutely no steering on the first bottom straight.
        # The controller publishes 0.0 steering until the car reaches this x position.
        self.declare_parameter('city_no_steering_until_x', 1.45)

        self.track_id = int(self.get_parameter('track_id').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.desired_lane_topic = str(self.get_parameter('desired_lane_topic').value)
        self.desired_yaw_topic = str(self.get_parameter('desired_yaw_topic').value)
        self.desired_cte_topic = str(self.get_parameter('desired_cte_topic').value)
        self.steering_lock_topic = str(self.get_parameter('steering_lock_topic').value)
        self.steering_cmd_topic = str(self.get_parameter('steering_cmd_topic').value)
        self.k = float(self.get_parameter('stanley_gain').value)
        self.city_k = float(self.get_parameter('city_stanley_gain').value)
        self.ks = float(self.get_parameter('softening_gain').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.city_heading_gain = float(self.get_parameter('city_heading_gain').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.minimum_speed = float(self.get_parameter('minimum_speed_for_stanley').value)
        self.steering_sign = float(self.get_parameter('steering_sign').value)
        self.alpha = float(self.get_parameter('steering_filter_alpha').value)
        self.max_steering_rate = float(self.get_parameter('max_steering_rate').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.city_no_steering_until_x = float(self.get_parameter('city_no_steering_until_x').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.desired_y = 0.0
        self.desired_yaw = 0.0
        self.desired_cte = 0.0
        self.steering_lock = 0.0
        self.filtered_steering = 0.0
        self.last_odom_time = None
        self.last_control_time = self.get_clock().now()

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Float64, self.desired_lane_topic, self.desired_lane_callback, 20)
        self.create_subscription(Float64, self.desired_yaw_topic, self.desired_yaw_callback, 20)
        self.create_subscription(Float64, self.desired_cte_topic, self.desired_cte_callback, 20)
        self.create_subscription(Float64, self.steering_lock_topic, self.steering_lock_callback, 20)
        self.pub = self.create_publisher(Float64, self.steering_cmd_topic, 20)
        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(f'MS5 Stanley lateral controller started. track_id={self.track_id}')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.sqrt(vx * vx + vy * vy)
        self.last_odom_time = self.get_clock().now()

    def desired_lane_callback(self, msg: Float64):
        self.desired_y = float(msg.data)

    def desired_yaw_callback(self, msg: Float64):
        self.desired_yaw = float(msg.data)

    def desired_cte_callback(self, msg: Float64):
        self.desired_cte = float(msg.data)

    def steering_lock_callback(self, msg: Float64):
        self.steering_lock = float(msg.data)

    def control_loop(self):
        now = self.get_clock().now()
        dt = max((now - self.last_control_time).nanoseconds / 1e9, 1e-3)
        self.last_control_time = now

        if self.last_odom_time is None or (now - self.last_odom_time).nanoseconds / 1e9 > self.odom_timeout:
            self.pub.publish(Float64(data=0.0))
            return

        # HARD LOCK for City Track bottom straight.
        # Planner owns the corner position. While /ms4/steering_lock = 1,
        # this controller ignores heading error, CTE, Stanley, and filters.
        if self.track_id == 3 and self.steering_lock > 0.5:
            self.filtered_steering = 0.0
            self.pub.publish(Float64(data=0.0))
            self.get_logger().info(
                f'[MS5 Stanley] TRACK 3 ABSOLUTE STEERING LOCK | x={self.x:.2f}, y={self.y:.2f} | steer=0.000',
                throttle_duration_sec=0.5)
            return

        effective_speed = max(self.speed, self.minimum_speed)
        heading_error = normalize_angle(self.desired_yaw - self.yaw)

        if self.track_id == 3:
            # For path-frame CTE: positive means the car is left of the path.
            # To return to the path, steering correction must be negative for positive CTE.
            cte = self.desired_cte
            cte_term = math.atan2(self.city_k * cte, effective_speed + self.ks)
            raw = self.steering_sign * (self.city_heading_gain * heading_error - cte_term)
        else:
            # Track 1/2 lane keeping: desired_y - actual_y.
            cte = self.desired_y - self.y
            cte_term = math.atan2(self.k * cte, effective_speed + self.ks)
            raw = self.steering_sign * (self.heading_gain * heading_error + cte_term)

        raw = clamp(raw, -self.max_steering_angle, self.max_steering_angle)

        max_step = self.max_steering_rate * dt
        rate_limited = clamp(raw, self.filtered_steering - max_step, self.filtered_steering + max_step)
        self.filtered_steering = self.alpha * rate_limited + (1.0 - self.alpha) * self.filtered_steering
        self.pub.publish(Float64(data=float(self.filtered_steering)))
        self.get_logger().info(
            f'[MS5 Stanley] track={self.track_id} x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, '
            f'desired_yaw={self.desired_yaw:.2f}, cte={cte:.3f}, steer={self.filtered_steering:.3f}',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MS5StanleyLateral()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
