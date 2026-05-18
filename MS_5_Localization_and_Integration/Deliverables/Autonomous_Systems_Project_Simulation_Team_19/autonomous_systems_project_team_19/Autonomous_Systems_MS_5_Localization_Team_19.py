#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .utils import yaw_from_quaternion, quaternion_from_yaw, normalize_angle


class MS5Localization(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Localization_Team_19')

        self.declare_parameter('ideal_odom_topic', '/model/vehicle_blue/odometry')
        self.declare_parameter('noisy_odom_topic', '/ms5/noisy_odom')
        self.declare_parameter('filtered_odom_topic', '/ms5/filtered_odom')

        self.declare_parameter('x_noise_std', 0.02)
        self.declare_parameter('y_noise_std', 0.02)
        self.declare_parameter('theta_noise_std', 0.01)
        self.declare_parameter('v_noise_std', 0.02)
        self.declare_parameter('omega_noise_std', 0.02)
        self.declare_parameter('process_noise_q', 0.003)
        self.declare_parameter('measurement_noise_r', 0.04)

        self.ideal_odom_topic = str(self.get_parameter('ideal_odom_topic').value)
        self.noisy_odom_topic = str(self.get_parameter('noisy_odom_topic').value)
        self.filtered_odom_topic = str(self.get_parameter('filtered_odom_topic').value)

        self.x_std = float(self.get_parameter('x_noise_std').value)
        self.y_std = float(self.get_parameter('y_noise_std').value)
        self.theta_std = float(self.get_parameter('theta_noise_std').value)
        self.v_std = float(self.get_parameter('v_noise_std').value)
        self.omega_std = float(self.get_parameter('omega_noise_std').value)
        q_value = float(self.get_parameter('process_noise_q').value)
        r_value = float(self.get_parameter('measurement_noise_r').value)

        # EKF state is [x, y, theta]. Inputs are noisy v and omega.
        self.x_hat = np.zeros((3, 1))
        self.P = np.eye(3) * 0.10
        self.Q = np.eye(3) * q_value
        self.R = np.eye(3) * r_value
        self.H = np.eye(3)
        self.initialized = False
        self.last_time = None

        self.noisy_pub = self.create_publisher(Odometry, self.noisy_odom_topic, 20)
        self.filtered_pub = self.create_publisher(Odometry, self.filtered_odom_topic, 20)
        self.create_subscription(Odometry, self.ideal_odom_topic, self.odom_callback, 20)

        self.get_logger().info('MS5 Localization started: ideal -> noisy -> Kalman filtered odometry')
        self.get_logger().info(f'Subscribing: {self.ideal_odom_topic}')
        self.get_logger().info(f'Publishing noisy: {self.noisy_odom_topic}')
        self.get_logger().info(f'Publishing filtered: {self.filtered_odom_topic}')

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        if self.last_time is None:
            dt = 0.02
        else:
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0.0 or dt > 0.25:
                dt = 0.02
        self.last_time = now

        x_real = msg.pose.pose.position.x
        y_real = msg.pose.pose.position.y
        theta_real = yaw_from_quaternion(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v_real = math.sqrt(vx * vx + vy * vy)
        omega_real = msg.twist.twist.angular.z

        x_noisy = x_real + np.random.normal(0.0, self.x_std)
        y_noisy = y_real + np.random.normal(0.0, self.y_std)
        theta_noisy = normalize_angle(theta_real + np.random.normal(0.0, self.theta_std))
        v_noisy = max(0.0, v_real + np.random.normal(0.0, self.v_std))
        omega_noisy = omega_real + np.random.normal(0.0, self.omega_std)

        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id
        noisy_msg.pose.pose.position.x = float(x_noisy)
        noisy_msg.pose.pose.position.y = float(y_noisy)
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z
        noisy_msg.pose.pose.orientation = quaternion_from_yaw(theta_noisy)
        noisy_msg.twist.twist.linear.x = float(v_noisy)
        noisy_msg.twist.twist.angular.z = float(omega_noisy)
        self.noisy_pub.publish(noisy_msg)

        if not self.initialized:
            self.x_hat = np.array([[x_noisy], [y_noisy], [theta_noisy]])
            self.initialized = True

        theta_est = self.x_hat[2, 0]
        x_pred = self.x_hat[0, 0] + v_noisy * math.cos(theta_est) * dt
        y_pred = self.x_hat[1, 0] + v_noisy * math.sin(theta_est) * dt
        theta_pred = normalize_angle(self.x_hat[2, 0] + omega_noisy * dt)
        x_pred_vec = np.array([[x_pred], [y_pred], [theta_pred]])

        F = np.array([
            [1.0, 0.0, -v_noisy * math.sin(theta_est) * dt],
            [0.0, 1.0,  v_noisy * math.cos(theta_est) * dt],
            [0.0, 0.0, 1.0]
        ])
        P_pred = F @ self.P @ F.T + self.Q

        z = np.array([[x_noisy], [y_noisy], [theta_noisy]])
        innovation = z - self.H @ x_pred_vec
        innovation[2, 0] = normalize_angle(innovation[2, 0])
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        self.x_hat = x_pred_vec + K @ innovation
        self.x_hat[2, 0] = normalize_angle(self.x_hat[2, 0])
        self.P = (np.eye(3) - K @ self.H) @ P_pred

        filtered_msg = Odometry()
        filtered_msg.header = msg.header
        filtered_msg.child_frame_id = msg.child_frame_id
        filtered_msg.pose.pose.position.x = float(self.x_hat[0, 0])
        filtered_msg.pose.pose.position.y = float(self.x_hat[1, 0])
        filtered_msg.pose.pose.position.z = msg.pose.pose.position.z
        filtered_msg.pose.pose.orientation = quaternion_from_yaw(float(self.x_hat[2, 0]))
        filtered_msg.twist.twist.linear.x = float(v_noisy)
        filtered_msg.twist.twist.angular.z = float(omega_noisy)
        self.filtered_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MS5Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
