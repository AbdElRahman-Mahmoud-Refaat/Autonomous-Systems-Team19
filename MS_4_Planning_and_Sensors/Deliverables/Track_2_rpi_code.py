#!/usr/bin/env python3
"""
Scenario 2 - Track 02 Lane Change - Team 19 - FINAL FINAL

Use this file by replacing the full content of:
~/ros2_ws/src/Autonomous_Systems_Project_Team_19/Autonomous_Systems_Project_Team_19/scenario2_track2_existing_arduino_team19.py

This version keeps the SAME low-level Arduino code.

Expected Arduino -> Raspberry Pi serial line:
    S,time_ms,encoder,ax,ay,az,gx,gy,gz,pwm,servo_us,imu_ok

Expected Raspberry Pi -> Arduino commands:
    C,<signed_pwm>,<servo_us>
    STOP
    ZERO
    ZEROENC
    ZEROIMU

What this node does on the Raspberry Pi:
    - Reads raw encoder and raw IMU data from Arduino.
    - Calculates speed, distance, x, y, and heading theta.
    - Plans Track 02 lane changes.
    - Uses Stanley controller for steering.
    - Sends smooth PWM and servo commands to Arduino.
    - Stops at the track end.

Controls:
    a = start autonomous Scenario 2
    s = emergency stop
    z = zero encoder + IMU + x/y/distance
    i = zero IMU heading only
    c = center steering
    q = stop and quit
"""

import math
import re
import sys
import time
import select
import termios
import tty

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool, String


class Scenario2Track2Team19(Node):
    def __init__(self):
        super().__init__('scenario2_track2_team19')

        # =========================
        # ROS parameters
        # =========================
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)

        # Encoder and vehicle calibration
        self.declare_parameter('ppr', 1085.0)
        self.declare_parameter('wheel_diameter_cm', 6.5)
        self.declare_parameter('encoder_sign', 1.0)
        self.declare_parameter('theta_sign', 1.0)
        self.declare_parameter('gyro_lsb_per_dps', 131.0)
        self.declare_parameter('calibration_samples', 60)

        # Keep this False so the code never refuses to start because of the old IMU flag issue.
        # It still reads and publishes imu_ok normally.
        self.declare_parameter('require_imu_ok', False)

        # Track 02 parameters
        self.declare_parameter('track_end_x_m', 10.0)
        self.declare_parameter('lane1_y_m', 0.1875)
        self.declare_parameter('lane2_y_m', -0.1875)
        self.declare_parameter('start_lane_end_x_m', 1.0)
        self.declare_parameter('change1_start_x_m', 2.60)
        self.declare_parameter('change1_end_x_m', 3.60)
        self.declare_parameter('change2_start_x_m', 6.60)
        self.declare_parameter('change2_end_x_m', 7.60)

        # Speed profile and motor command
        self.declare_parameter('normal_speed_mps', 0.18)
        self.declare_parameter('lane_change_speed_mps', 0.11)
        self.declare_parameter('final_speed_mps', 0.08)
        self.declare_parameter('slow_down_distance_m', 0.45)
        self.declare_parameter('min_move_cmd', 75.0)
        self.declare_parameter('max_motor_cmd', 170.0)
        self.declare_parameter('speed_kp_pwm_per_mps', 250.0)
        self.declare_parameter('pwm_ramp_per_sec', 700.0)

        # Stanley controller and servo
        self.declare_parameter('k_stanley', 0.75)
        self.declare_parameter('eps_speed', 0.05)
        self.declare_parameter('servo_center_us', 1500)
        self.declare_parameter('servo_min_us', 1000)
        self.declare_parameter('servo_max_us', 2000)
        self.declare_parameter('us_per_deg', 8.0)
        self.declare_parameter('max_steer_deg', 23.0)
        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('cte_sign', 1.0)

        # Debug
        self.declare_parameter('print_raw', False)

        # =========================
        # Read parameters
        # =========================
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)

        self.ppr = float(self.get_parameter('ppr').value)
        self.wheel_diameter_cm = float(self.get_parameter('wheel_diameter_cm').value)
        self.wheel_circumference_m = math.pi * self.wheel_diameter_cm / 100.0
        self.encoder_sign = float(self.get_parameter('encoder_sign').value)
        self.theta_sign = float(self.get_parameter('theta_sign').value)
        self.gyro_lsb_per_dps = float(self.get_parameter('gyro_lsb_per_dps').value)
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)
        self.require_imu_ok = bool(self.get_parameter('require_imu_ok').value)

        self.track_end_x_m = float(self.get_parameter('track_end_x_m').value)
        self.lane1_y_m = float(self.get_parameter('lane1_y_m').value)
       self.lane2_y_m = float(self.get_parameter('lane2_y_m').value)
        self.start_lane_end_x_m = float(self.get_parameter('start_lane_end_x_m').value)
        self.change1_start_x_m = float(self.get_parameter('change1_start_x_m').value)
        self.change1_end_x_m = float(self.get_parameter('change1_end_x_m').value)
        self.change2_start_x_m = float(self.get_parameter('change2_start_x_m').value)
        self.change2_end_x_m = float(self.get_parameter('change2_end_x_m').value)

        self.normal_speed_mps = float(self.get_parameter('normal_speed_mps').value)
        self.lane_change_speed_mps = float(self.get_parameter('lane_change_speed_mps').value)
        self.final_speed_mps = float(self.get_parameter('final_speed_mps').value)
        self.slow_down_distance_m = float(self.get_parameter('slow_down_distance_m').value)
        self.min_move_cmd = float(self.get_parameter('min_move_cmd').value)
        self.max_motor_cmd = float(self.get_parameter('max_motor_cmd').value)
        self.speed_kp_pwm_per_mps = float(self.get_parameter('speed_kp_pwm_per_mps').value)
        self.pwm_ramp_per_sec = float(self.get_parameter('pwm_ramp_per_sec').value)

        self.k_stanley = float(self.get_parameter('k_stanley').value)
        self.eps_speed = float(self.get_parameter('eps_speed').value)
        self.servo_center_us = int(self.get_parameter('servo_center_us').value)
        self.servo_min_us = int(self.get_parameter('servo_min_us').value)
        self.servo_max_us = int(self.get_parameter('servo_max_us').value)
        self.us_per_deg = float(self.get_parameter('us_per_deg').value)
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.cte_sign = float(self.get_parameter('cte_sign').value)

        self.print_raw = bool(self.get_parameter('print_raw').value)

        # =========================
        # Serial
        # =========================
        self.ser = serial.Serial(self.port, self.baud, timeout=0.001, write_timeout=0.1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.rx_buffer = ''

        self.state_regex = re.compile(
            r'S\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'

            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)\s*,\s*'
            r'(-?\d+)'
        )

        # =========================
        # Publishers
        # =========================
        self.raw_pub = self.create_publisher(String, '/vehicle/state_raw', 10)
        self.cmd_pub = self.create_publisher(String, '/vehicle/cmd', 10)
        self.mode_pub = self.create_publisher(String, '/scenario2/mode', 10)
        self.summary_pub = self.create_publisher(String, '/scenario2/summary', 10)

        self.encoder_pub = self.create_publisher(Int32, '/vehicle/encoder_count', 10)
        self.imu_ok_pub = self.create_publisher(Bool, '/vehicle/imu_ok', 10)

        self.x_pub = self.create_publisher(Float32, '/vehicle/x_m', 10)
        self.y_pub = self.create_publisher(Float32, '/vehicle/y_m', 10)
        self.theta_pub = self.create_publisher(Float32, '/vehicle/imu_theta_deg', 10)
        self.speed_pub = self.create_publisher(Float32, '/vehicle/speed_mps', 10)
        self.distance_pub = self.create_publisher(Float32, '/vehicle/abs_distance_m', 10)

        self.lane_ref_pub = self.create_publisher(Float32, '/scenario2/lane_ref_m', 10)
        self.cte_pub = self.create_publisher(Float32, '/scenario2/crosstrack_error_m', 10)
        self.heading_ref_pub = self.create_publisher(Float32, '/scenario2/heading_ref_deg', 10)
        self.heading_error_pub = self.create_publisher(Float32, '/scenario2/heading_error_deg', 10)
        self.desired_speed_pub = self.create_publisher(Float32, '/scenario2/desired_speed_mps', 10)

        self.motor_cmd_pub = self.create_publisher(Float32, '/vehicle/motor_target_cmd', 10)
        self.servo_us_pub = self.create_publisher(Int32, '/vehicle/servo_us', 10)
        self.steer_deg_pub = self.create_publisher(Float32, '/vehicle/steering_delta_deg', 10)

        # =========================
        # State variables
        # =========================
       self.x_m = 0.0
        self.y_m = 0.0
        self.theta_deg = 0.0
        self.theta_rad = 0.0
        self.speed_mps = 0.0
        self.raw_speed_mps = 0.0
        self.distance_m = 0.0

        self.prev_time_ms = None
        self.prev_encoder_raw = None
        self.last_encoder_raw = 0
        self.last_state_ok = False
        self.imu_ok = False
        self.bad_imu_count = 0

        self.gz_offset = 0.0
        self.gz_offset_sum = 0.0
        self.gz_offset_count = 0
        self.calibrating = True

        self.auto_enabled = False
        self.finished = False
        self.mode = 'CALIBRATING'

        self.desired_lane_y_m = 0.0
        self.desired_heading_rad = 0.0
        self.desired_heading_deg = 0.0
        self.cross_track_error_m = 0.0
        self.heading_error_deg = 0.0
        self.desired_speed_mps = 0.0

        self.target_pwm = 0.0
        self.cmd_pwm = 0.0
        self.last_control_time = time.monotonic()

        self.servo_us_cmd = self.servo_center_us
        self.steer_deg_cmd = 0.0
        self.arduino_pwm = 0
        self.arduino_servo_us = self.servo_center_us

        self.last_dashboard_time = 0.0
       self.last_warn_time = 0.0

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(f'Scenario 2 final node started on {self.port} @ {self.baud}')
        self.print_instructions()

    def print_instructions(self):
        print('\nScenario 2 - Track 02 Lane Change - FINAL FINAL')
        print('----------------------------------------------')
        print('Controls:')
        print('  a : start autonomous Scenario 2')
        print('  s : emergency stop')
        print('  z : zero encoder, IMU heading, x, y, distance')
        print('  i : zero IMU heading only')
        print('  c : center steering')
        print('  q : stop and quit')
        print('')
        print('Start sequence:')
        print('  1) Keep car still until gyro calibration finishes.')
        print('  2) Put car at start line.')
        print('  3) Press z.')
        print('  4) Press a once.')
        print('')

    # =========================
    # Helpers
    # =========================
    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def normalize_angle_rad(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def smoothstep(self, t):
        t = self.clamp(t, 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)
   def smoothstep_derivative(self, t):
        t = self.clamp(t, 0.0, 1.0)
        return 6.0 * t * (1.0 - t)

    def publish_float(self, pub, value):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def publish_int(self, pub, value):
        msg = Int32()
        msg.data = int(value)
        pub.publish(msg)

    def publish_bool(self, pub, value):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def publish_string(self, pub, value):
        msg = String()
        msg.data = str(value)
        pub.publish(msg)

    # =========================
    # Arduino communication
    # =========================
    def send_line(self, line):
        try:
            self.ser.write((line.strip() + '\n').encode('utf-8'))
            self.ser.flush()
        except Exception as e:
            self.get_logger().warn(f'Failed to write serial command: {e}')

    def send_to_arduino(self, pwm, servo_us):
        pwm = int(self.clamp(pwm, -255, 255))
        servo_us = int(self.clamp(servo_us, self.servo_min_us, self.servo_max_us))
        cmd = f'C,{pwm},{servo_us}'
        self.send_line(cmd)
        self.publish_string(self.cmd_pub, cmd)


   def stop_car(self, center=True):
        self.auto_enabled = False
        self.desired_speed_mps = 0.0
        self.target_pwm = 0.0
        self.cmd_pwm = 0.0
        servo = self.servo_center_us if center else self.servo_us_cmd
        self.send_to_arduino(0, servo)
        self.send_line('STOP')

    def center_steering(self):
        self.servo_us_cmd = self.servo_center_us
        self.steer_deg_cmd = 0.0
        self.send_to_arduino(0 if not self.auto_enabled else self.cmd_pwm, self.servo_center_us)
        self.get_logger().info('Steering centered')

    def zero_heading_only(self):
        self.theta_deg = 0.0
        self.theta_rad = 0.0
        self.send_line('ZEROIMU')
        self.get_logger().info('IMU heading zeroed')

    def zero_all(self):
        self.stop_car(center=True)

        self.x_m = 0.0
        self.y_m = 0.0
        self.theta_deg = 0.0
        self.theta_rad = 0.0
        self.speed_mps = 0.0
        self.raw_speed_mps = 0.0
        self.distance_m = 0.0

        self.prev_time_ms = None
        self.prev_encoder_raw = None

        self.desired_lane_y_m = 0.0
        self.desired_heading_rad = 0.0
        self.desired_heading_deg = 0.0
        self.cross_track_error_m = 0.0
        self.heading_error_deg = 0.0
        self.desired_speed_mps = 0.0

        self.target_pwm = 0.0
        self.cmd_pwm = 0.0
        self.servo_us_cmd = self.servo_center_us
        self.steer_deg_cmd = 0.0

        self.send_line('ZERO')
        time.sleep(0.03)
        self.send_line('ZEROENC')
        time.sleep(0.03)
        self.send_line('ZEROIMU')
        time.sleep(0.03)
        self.send_to_arduino(0, self.servo_center_us)

        self.mode = 'WAITING'
        self.finished = False
        self.get_logger().info('Zeroed Scenario 2 states + Arduino encoder/IMU')

    # =========================
    # Parse Arduino state
    # =========================
    def parse_state_line(self, line):
        m = self.state_regex.search(line)
        if not m:
            return None

        try:
            values = [int(v) for v in m.groups()]
            return {
                'time_ms': values[0],
                'encoder': values[1],
                'ax': values[2],
                'ay': values[3],
                'az': values[4],
                'gx': values[5],
                'gy': values[6],
                'gz': values[7],
                'arduino_pwm': values[8],
                'arduino_servo': values[9],
                'imu_ok': values[10],
            }
        except Exception:
           return None

    # =========================
    # IMU calibration and localization
    # =========================
    def update_gyro_calibration(self, gz_raw):
        gz_dps = gz_raw / self.gyro_lsb_per_dps
        self.gz_offset_sum += gz_dps
        self.gz_offset_count += 1

        if self.gz_offset_count >= self.calibration_samples:
            self.gz_offset = self.gz_offset_sum / max(1, self.gz_offset_count)
            self.calibrating = False
            self.mode = 'WAITING'
            self.get_logger().info(f'Gyro calibration done. gz_offset = {self.gz_offset:.4f} deg/s')
            self.get_logger().info('Press z at start line, then press a to start.')

    def update_localization(self, state):
        time_ms = state['time_ms']
        encoder_raw = state['encoder']
        gz_raw = state['gz']

        self.last_encoder_raw = encoder_raw

        if self.prev_time_ms is None or self.prev_encoder_raw is None:
            self.prev_time_ms = time_ms
            self.prev_encoder_raw = encoder_raw
            return

        dt = (time_ms - self.prev_time_ms) / 1000.0
        delta_counts_raw = encoder_raw - self.prev_encoder_raw

        self.prev_time_ms = time_ms
        self.prev_encoder_raw = encoder_raw

        if dt <= 0.0001 or dt > 1.0:
            return

        delta_counts = self.encoder_sign * float(delta_counts_raw)
        delta_distance_m = (delta_counts / self.ppr) * self.wheel_circumference_m

       self.raw_speed_mps = delta_distance_m / dt
        self.speed_mps = 0.65 * self.speed_mps + 0.35 * self.raw_speed_mps
        self.distance_m += delta_distance_m

        gz_dps = ((gz_raw / self.gyro_lsb_per_dps) - self.gz_offset) * self.theta_sign
        self.theta_deg += gz_dps * dt

        while self.theta_deg > 180.0:
            self.theta_deg -= 360.0
        while self.theta_deg < -180.0:
            self.theta_deg += 360.0

        self.theta_rad = math.radians(self.theta_deg)

        self.x_m += delta_distance_m * math.cos(self.theta_rad)
        self.y_m += delta_distance_m * math.sin(self.theta_rad)

    # =========================
    # Track 02 planner
    # =========================
    def interpolate_lane(self, x, x0, x1, y0, y1):
        length = max(0.001, x1 - x0)
        t = (x - x0) / length
        s = self.smoothstep(t)
        dsdt = self.smoothstep_derivative(t)
        y = y0 + (y1 - y0) * s
        dy_dx = (y1 - y0) * dsdt / length
        return y, dy_dx

    def lane_profile(self, x):
        if x < self.start_lane_end_x_m:
            return self.interpolate_lane(x, 0.0, self.start_lane_end_x_m, 0.0, self.lane1_y_m)

        if x < self.change1_start_x_m:
            return self.lane1_y_m, 0.0

        if x < self.change1_end_x_m:
            return self.interpolate_lane(
                x,
                self.change1_start_x_m,
                self.change1_end_x_m,
                self.lane2_y_m,
                self.lane1_y_m
            )

        return self.lane1_y_m, 0.0

    def update_planner(self):
        self.desired_lane_y_m, dy_dx = self.lane_profile(self.x_m)
        self.desired_heading_rad = math.atan(dy_dx)
        self.desired_heading_deg = math.degrees(self.desired_heading_rad)

        in_lane_change = (
            self.x_m < self.start_lane_end_x_m or
            self.change1_start_x_m <= self.x_m < self.change1_end_x_m or
            self.change2_start_x_m <= self.x_m < self.change2_end_x_m
        )

        base_speed = self.lane_change_speed_mps if in_lane_change else self.normal_speed_mps

        remaining = self.track_end_x_m - self.x_m
        if remaining <= 0.0:
            self.desired_speed_mps = 0.0
            self.auto_enabled = False
            self.finished = True
            self.mode = 'FINISHED'
            self.stop_car(center=True)
            self.get_logger().info('Scenario 2 finished: reached track end.')
            return

        if remaining < self.slow_down_distance_m:
            ratio = self.clamp(remaining / self.slow_down_distance_m, 0.0, 1.0)
            slow_speed = self.final_speed_mps + (base_speed - self.final_speed_mps) * ratio
            self.desired_speed_mps = max(0.04, min(base_speed, slow_speed))
        else:
            self.desired_speed_mps = base_speed

    # =========================
    # Controllers
    # =========================
    def update_speed_controller(self):
        now = time.monotonic()
       dt = now - self.last_control_time
        self.last_control_time = now
        dt = self.clamp(dt, 0.001, 0.2)

        if self.desired_speed_mps <= 0.001:
            self.target_pwm = 0.0
            self.cmd_pwm = 0.0
            return 0

        speed_ratio = self.clamp(self.desired_speed_mps / max(0.01, self.normal_speed_mps), 0.0, 1.0)
        open_loop_pwm = self.min_move_cmd + speed_ratio * (self.max_motor_cmd - self.min_move_cmd)

        measured_forward_speed = max(0.0, self.speed_mps)
        speed_error = self.desired_speed_mps - measured_forward_speed
        feedback_pwm = self.speed_kp_pwm_per_mps * speed_error

        self.target_pwm = self.clamp(open_loop_pwm + feedback_pwm, self.min_move_cmd, self.max_motor_cmd)

        max_step = self.pwm_ramp_per_sec * dt
        if self.cmd_pwm < self.target_pwm:
            self.cmd_pwm = min(self.cmd_pwm + max_step, self.target_pwm)
        elif self.cmd_pwm > self.target_pwm:
            self.cmd_pwm = max(self.cmd_pwm - max_step, self.target_pwm)

        return int(self.clamp(round(self.cmd_pwm), 0, self.max_motor_cmd))

    def update_stanley_controller(self):
        self.cross_track_error_m = self.cte_sign * (self.desired_lane_y_m - self.y_m)
        heading_error_rad = self.normalize_angle_rad(self.desired_heading_rad - self.theta_rad)
        self.heading_error_deg = math.degrees(heading_error_rad)

        speed_for_control = max(abs(self.speed_mps), self.eps_speed)
        cte_term_rad = math.atan2(self.k_stanley * self.cross_track_error_m, speed_for_control)

        steering_rad = heading_error_rad + cte_term_rad
        steering_deg = math.degrees(steering_rad)
        steering_deg = self.clamp(steering_deg, -self.max_steer_deg, self.max_steer_deg)

        self.steer_deg_cmd = steering_deg
        servo_us = self.servo_center_us + self.steer_sign * self.us_per_deg * steering_deg
        servo_us = int(self.clamp(round(servo_us), self.servo_min_us, self.servo_max_us))
       self.servo_us_cmd = servo_us
        return servo_us

    # =========================
    # Main processing
    # =========================
    def process_state(self, state):
        self.last_state_ok = True
        self.imu_ok = (int(state.get('imu_ok', 0)) == 1)
        self.arduino_pwm = int(state.get('arduino_pwm', 0))
        self.arduino_servo_us = int(state.get('arduino_servo', self.servo_center_us))

        if self.imu_ok:
            self.bad_imu_count = 0
        else:
            self.bad_imu_count += 1
            now = time.monotonic()
            if now - self.last_warn_time > 0.7:
                self.get_logger().warn(f'Latest Arduino packet has imu_ok=0. bad_count={self.bad_imu_count}')
                self.last_warn_time = now
            if self.require_imu_ok and self.auto_enabled and self.bad_imu_count >= 5:
                self.get_logger().warn('IMU not OK for several packets. Emergency stop.')
                self.emergency_stop()
                return

        if self.calibrating:
            self.mode = 'CALIBRATING'
            self.update_gyro_calibration(state['gz'])
            self.send_to_arduino(0, self.servo_center_us)
            self.publish_all(state)
            self.print_dashboard()
            return

        self.update_localization(state)

        if self.auto_enabled:
            self.mode = 'AUTO'
            self.update_planner()
            if self.auto_enabled:
                pwm = self.update_speed_controller()
                servo = self.update_stanley_controller()
               self.send_to_arduino(pwm, servo)
        else:
            if not self.finished:
                self.mode = 'WAITING'
            self.send_to_arduino(0, self.servo_center_us)

        self.publish_all(state)
        self.print_dashboard()

    def timer_callback(self):
        latest_state = None

        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                data = self.ser.read(waiting).decode('utf-8', errors='ignore')
                if data:
                    self.rx_buffer += data

                if len(self.rx_buffer) > 3000:
                    self.rx_buffer = self.rx_buffer[-1000:]

                while '\n' in self.rx_buffer:
                    line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                    line = line.replace('\r', '').strip()
                    if not line:
                        continue

                    self.publish_string(self.raw_pub, line)

                    if self.print_raw:
                        print(line)

                    state = self.parse_state_line(line)
                    if state is not None:
                        latest_state = state

        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return
       if latest_state is not None:
            self.process_state(latest_state)

    # =========================
    # Publish and dashboard
    # =========================
    def publish_all(self, state):
        self.publish_string(self.mode_pub, self.mode)
        self.publish_int(self.encoder_pub, state['encoder'])
        self.publish_bool(self.imu_ok_pub, self.imu_ok)

        self.publish_float(self.x_pub, self.x_m)
        self.publish_float(self.y_pub, self.y_m)
        self.publish_float(self.theta_pub, self.theta_deg)
        self.publish_float(self.speed_pub, self.speed_mps)
        self.publish_float(self.distance_pub, abs(self.distance_m))

        self.publish_float(self.lane_ref_pub, self.desired_lane_y_m)
        self.publish_float(self.cte_pub, self.cross_track_error_m)
        self.publish_float(self.heading_ref_pub, self.desired_heading_deg)
        self.publish_float(self.heading_error_pub, self.heading_error_deg)
        self.publish_float(self.desired_speed_pub, self.desired_speed_mps)

        self.publish_float(self.motor_cmd_pub, self.cmd_pwm)
        self.publish_int(self.servo_us_pub, self.servo_us_cmd)
        self.publish_float(self.steer_deg_pub, self.steer_deg_cmd)

        summary = (
            f'mode={self.mode}, '
            f'x={self.x_m:.3f}, y={self.y_m:.3f}, d={abs(self.distance_m):.3f}, '
            f'theta={self.theta_deg:.2f}, v={self.speed_mps:.3f}, '
            f'lane_ref={self.desired_lane_y_m:.4f}, cte={self.cross_track_error_m:.4f}, '
            f'v_ref={self.desired_speed_mps:.3f}, pwm={self.cmd_pwm:.1f}, '
            f'steer={self.steer_deg_cmd:.2f}, imu={int(self.imu_ok)}'
        )
        self.publish_string(self.summary_pub, summary)

    def print_dashboard(self):
        now = time.monotonic()
        if now - self.last_dashboard_time < 0.20:
            return
       if latest_state is not None:
            self.process_state(latest_state)

    # =========================
    # Publish and dashboard
    # =========================
    def publish_all(self, state):
        self.publish_string(self.mode_pub, self.mode)
        self.publish_int(self.encoder_pub, state['encoder'])
        self.publish_bool(self.imu_ok_pub, self.imu_ok)

        self.publish_float(self.x_pub, self.x_m)
        self.publish_float(self.y_pub, self.y_m)
        self.publish_float(self.theta_pub, self.theta_deg)
        self.publish_float(self.speed_pub, self.speed_mps)
        self.publish_float(self.distance_pub, abs(self.distance_m))

        self.publish_float(self.lane_ref_pub, self.desired_lane_y_m)
        self.publish_float(self.cte_pub, self.cross_track_error_m)
        self.publish_float(self.heading_ref_pub, self.desired_heading_deg)
        self.publish_float(self.heading_error_pub, self.heading_error_deg)
        self.publish_float(self.desired_speed_pub, self.desired_speed_mps)

        self.publish_float(self.motor_cmd_pub, self.cmd_pwm)
        self.publish_int(self.servo_us_pub, self.servo_us_cmd)
        self.publish_float(self.steer_deg_pub, self.steer_deg_cmd)

        summary = (
            f'mode={self.mode}, '
            f'x={self.x_m:.3f}, y={self.y_m:.3f}, d={abs(self.distance_m):.3f}, '
            f'theta={self.theta_deg:.2f}, v={self.speed_mps:.3f}, '
            f'lane_ref={self.desired_lane_y_m:.4f}, cte={self.cross_track_error_m:.4f}, '
            f'v_ref={self.desired_speed_mps:.3f}, pwm={self.cmd_pwm:.1f}, '
            f'steer={self.steer_deg_cmd:.2f}, imu={int(self.imu_ok)}'
        )
        self.publish_string(self.summary_pub, summary)

    def print_dashboard(self):
        now = time.monotonic()
        if now - self.last_dashboard_time < 0.20:
            return
       if latest_state is not None:
            self.process_state(latest_state)

    # =========================
    # Publish and dashboard
    # =========================
    def publish_all(self, state):
        self.publish_string(self.mode_pub, self.mode)
        self.publish_int(self.encoder_pub, state['encoder'])
        self.publish_bool(self.imu_ok_pub, self.imu_ok)

        self.publish_float(self.x_pub, self.x_m)
        self.publish_float(self.y_pub, self.y_m)
        self.publish_float(self.theta_pub, self.theta_deg)
        self.publish_float(self.speed_pub, self.speed_mps)
        self.publish_float(self.distance_pub, abs(self.distance_m))

        self.publish_float(self.lane_ref_pub, self.desired_lane_y_m)
        self.publish_float(self.cte_pub, self.cross_track_error_m)
        self.publish_float(self.heading_ref_pub, self.desired_heading_deg)
        self.publish_float(self.heading_error_pub, self.heading_error_deg)
        self.publish_float(self.desired_speed_pub, self.desired_speed_mps)

        self.publish_float(self.motor_cmd_pub, self.cmd_pwm)
        self.publish_int(self.servo_us_pub, self.servo_us_cmd)
        self.publish_float(self.steer_deg_pub, self.steer_deg_cmd)

        summary = (
            f'mode={self.mode}, '
            f'x={self.x_m:.3f}, y={self.y_m:.3f}, d={abs(self.distance_m):.3f}, '
            f'theta={self.theta_deg:.2f}, v={self.speed_mps:.3f}, '
            f'lane_ref={self.desired_lane_y_m:.4f}, cte={self.cross_track_error_m:.4f}, '
            f'v_ref={self.desired_speed_mps:.3f}, pwm={self.cmd_pwm:.1f}, '
            f'steer={self.steer_deg_cmd:.2f}, imu={int(self.imu_ok)}'
        )
        self.publish_string(self.summary_pub, summary)

    def print_dashboard(self):
        now = time.monotonic()
        if now - self.last_dashboard_time < 0.20:
            return
       if latest_state is not None:
            self.process_state(latest_state)

    # =========================
    # Publish and dashboard
    # =========================
    def publish_all(self, state):
        self.publish_string(self.mode_pub, self.mode)
        self.publish_int(self.encoder_pub, state['encoder'])
        self.publish_bool(self.imu_ok_pub, self.imu_ok)

        self.publish_float(self.x_pub, self.x_m)
        self.publish_float(self.y_pub, self.y_m)
        self.publish_float(self.theta_pub, self.theta_deg)
        self.publish_float(self.speed_pub, self.speed_mps)
        self.publish_float(self.distance_pub, abs(self.distance_m))

        self.publish_float(self.lane_ref_pub, self.desired_lane_y_m)
        self.publish_float(self.cte_pub, self.cross_track_error_m)
        self.publish_float(self.heading_ref_pub, self.desired_heading_deg)
        self.publish_float(self.heading_error_pub, self.heading_error_deg)
        self.publish_float(self.desired_speed_pub, self.desired_speed_mps)

        self.publish_float(self.motor_cmd_pub, self.cmd_pwm)
        self.publish_int(self.servo_us_pub, self.servo_us_cmd)
        self.publish_float(self.steer_deg_pub, self.steer_deg_cmd)

        summary = (
            f'mode={self.mode}, '
            f'x={self.x_m:.3f}, y={self.y_m:.3f}, d={abs(self.distance_m):.3f}, '
            f'theta={self.theta_deg:.2f}, v={self.speed_mps:.3f}, '
            f'lane_ref={self.desired_lane_y_m:.4f}, cte={self.cross_track_error_m:.4f}, '
            f'v_ref={self.desired_speed_mps:.3f}, pwm={self.cmd_pwm:.1f}, '
            f'steer={self.steer_deg_cmd:.2f}, imu={int(self.imu_ok)}'
        )
        self.publish_string(self.summary_pub, summary)

    def print_dashboard(self):
        now = time.monotonic()
        if now - self.last_dashboard_time < 0.20:
            return
        self.last_dashboard_time = now

        print(
            '\r'
            f'mode={self.mode:<11} | '
            f'x={self.x_m:5.2f}m y={self.y_m:+6.3f}m d={abs(self.distance_m):5.2f}m | '
            f'lane={self.desired_lane_y_m:+6.3f} cte={self.cross_track_error_m:+6.3f} | '
            f'theta={self.theta_deg:+6.2f}deg href={self.desired_heading_deg:+5.1f} | '
            f'v={self.speed_mps:+5.2f} vref={self.desired_speed_mps:4.2f} | '
            f'T={self.target_pwm:5.1f} C={self.cmd_pwm:5.1f} Ard={self.arduino_pwm:4d} | '
            f'steer={self.steer_deg_cmd:+5.1f}deg servo={self.servo_us_cmd:4d} | '
            f'IMU={int(self.imu_ok)} | a start s stop z zero q quit   ',
            end='',
            flush=True
        )

    # =========================
    # Keyboard actions
    # =========================
    def start_scenario(self):
        if not self.last_state_ok:
            self.get_logger().warn('Cannot start: no valid Arduino S packet received yet.')
            return

        if self.calibrating:
            self.get_logger().warn('Cannot start: gyro calibration is still running. Keep car still.')
            return

        if self.require_imu_ok and not self.imu_ok:
            self.get_logger().warn('Cannot start: latest Arduino packet has imu_ok=0.')
            return

        self.zero_all()
        time.sleep(0.10)

        self.auto_enabled = True
        self.finished = False
        self.mode = 'AUTO'
        self.last_control_time = time.monotonic()
        self.target_pwm = 0.0
        self.cmd_pwm = 0.0
       self.get_logger().info('Scenario 2 started.')

    def emergency_stop(self):
        self.stop_car(center=True)
        self.auto_enabled = False
        self.mode = 'STOPPED'
        self.get_logger().warn('Emergency stop sent.')

    def shutdown_safe(self):
        try:
            self.stop_car(center=True)
            time.sleep(0.05)
            self.send_line('STOP')
            time.sleep(0.05)
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


# =========================
# Keyboard helper
# =========================
def get_key(timeout=0.02):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    key = ''
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


# =========================
# Main
# =========================
def main(args=None):
   rclpy.init(args=args)
    node = None

    try:
        node = Scenario2Track2Team19()

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.002)
            key = get_key(timeout=0.005)

            if not key:
                continue

            if key.lower() == 'a':
                node.start_scenario()
            elif key.lower() == 's':
                node.emergency_stop()
            elif key.lower() == 'z':
                node.zero_all()
            elif key.lower() == 'i':
                node.zero_heading_only()
            elif key.lower() == 'c':
                node.center_steering()
            elif key.lower() == 'q':
                node.get_logger().info('Quit requested.')
                break

    except KeyboardInterrupt:
        pass
    finally:
        print('\nStopping Scenario 2 node safely...')
        if node is not None:
            node.shutdown_safe()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()