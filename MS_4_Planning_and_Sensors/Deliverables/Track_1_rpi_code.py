#!/usr/bin/env python3
"""
TEAM 19 - Scenario 1: Straight-Line Autonomous Run to 4 m

Architecture:
- Arduino is low-level only: motor PWM, servo pulse, encoder count, raw MPU6050.
- Raspberry Pi performs all processing and control:
    * encoder distance / speed / odometry
    * IMU yaw integration
    * heading-hold steering controller
    * stop condition at target distance

Arduino serial protocol expected:
- Command to Arduino: C,<signed_pwm>,<servo_us>
- Text commands: STOP, ZERO
- State from Arduino: S,time_ms,encoder,ax,ay,az,gx,gy,gz,pwm,servo_us,imu_ok
"""

import math
import re
import sys
import time
import termios
import tty
import select
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, Int32, String


STATE_RE = re.compile(
    r"^S,(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(>
)


HELP = """
TEAM 19 - Scenario 1: Straight Line to 4 m
------------------------------------------------
g : start scenario
s : emergency stop and keep node open
z : zero encoder / distance / x / y before start
i : zero IMU theta before start
q : stop and quit

Control logic:
- Encoder distance stops the vehicle at the target distance.
- IMU theta is used to maintain a straight heading.
- Steering correction is automatic.
- Arduino remains low-level only.
"""


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def angle_error_deg(target, current):
    return wrap_angle_deg(target - current)


def get_key(timeout=0.001):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return ""
    ch1 = sys.stdin.read(1)
    if ch1 == "\x1b":
        # consume possible arrow sequence, but scenario node does not use arrows
        seq = ch1
        while select.select([sys.stdin], [], [], 0.001)[0]:
            seq += sys.stdin.read(1)
            if len(seq) >= 3:
                break
        return seq
    return ch1


class Scenario1Straight4mNode(Node):
    def __init__(self):
        super().__init__('scenario1_straight_4m_team19')

        # Serial
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)

        # Calibrated vehicle parameters
        # PPR was calibrated from 1.20 m real vs 3.48 m displayed using old 374 PPR.
        self.declare_parameter('ppr', 1085.0)
        self.declare_parameter('wheel_diameter_cm', 6.5)
        self.declare_parameter('encoder_sign', 1.0)

        # Scenario parameters
        self.declare_parameter('target_distance_m', 4.0)
        self.declare_parameter('stop_tolerance_m', 0.02)
        self.declare_parameter('cruise_pwm', 220.0)
        self.declare_parameter('min_move_pwm', 75.0)
        self.declare_parameter('max_pwm', 255.0)
  self.declare_parameter('slowdown_distance_m', 0.80)
        self.declare_parameter('final_slow_distance_m', 0.20)
        self.declare_parameter('final_slow_pwm', 85.0)
        self.declare_parameter('pwm_ramp_per_sec', 500.0)
        self.declare_parameter('max_run_time_s', 60.0)

        # Steering / heading controller
        self.declare_parameter('steer_center_us', 1500)
        self.declare_parameter('steer_min_us', 1000)
        self.declare_parameter('steer_max_us', 2000)
        self.declare_parameter('steer_max_deg', 30.0)
        self.declare_parameter('servo_trim_us', 0.0)
        self.declare_parameter('heading_kp_us_per_deg', 16.0)
        self.declare_parameter('heading_kd_us_per_dps', 2.0)
        self.declare_parameter('heading_deadband_deg', 0.4)
        self.declare_parameter('max_steer_correction_us', 220.0)
        self.declare_parameter('servo_rate_us_per_sec', 900.0)

        # Processing
        self.declare_parameter('speed_filter_alpha', 0.35)
        self.declare_parameter('gyro_calib_seconds', 2.0)
        self.declare_parameter('cmd_rate_hz', 30.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.ppr = float(self.get_parameter('ppr').value)
        self.wheel_diameter_cm = float(self.get_parameter('wheel_diameter_cm').value)
        self.encoder_sign = float(self.get_parameter('encoder_sign').value)
        self.wheel_circ_m = math.pi * (self.wheel_diameter_cm / 100.0)

        self.target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.stop_tolerance_m = float(self.get_parameter('stop_tolerance_m').value)
        self.cruise_pwm = float(self.get_parameter('cruise_pwm').value)
        self.min_move_pwm = float(self.get_parameter('min_move_pwm').value)
        self.max_pwm = float(self.get_parameter('max_pwm').value)
  self.slowdown_distance_m = float(self.get_parameter('slowdown_distance_m').value)
        self.final_slow_distance_m = float(self.get_parameter('final_slow_distance_m').val>
        self.final_slow_pwm = float(self.get_parameter('final_slow_pwm').value)
        self.pwm_ramp_per_sec = float(self.get_parameter('pwm_ramp_per_sec').value)
        self.max_run_time_s = float(self.get_parameter('max_run_time_s').value)

        self.steer_center_us = int(self.get_parameter('steer_center_us').value)
        self.steer_min_us = int(self.get_parameter('steer_min_us').value)
        self.steer_max_us = int(self.get_parameter('steer_max_us').value)
        self.steer_max_deg = float(self.get_parameter('steer_max_deg').value)
        self.servo_trim_us = float(self.get_parameter('servo_trim_us').value)
        self.heading_kp = float(self.get_parameter('heading_kp_us_per_deg').value)
        self.heading_kd = float(self.get_parameter('heading_kd_us_per_dps').value)
        self.heading_deadband = float(self.get_parameter('heading_deadband_deg').value)
        self.max_steer_correction_us = float(self.get_parameter('max_steer_correction_us')>
        self.servo_rate_us_per_sec = float(self.get_parameter('servo_rate_us_per_sec').val>

        self.speed_alpha = float(self.get_parameter('speed_filter_alpha').value)
        self.gyro_calib_seconds = float(self.get_parameter('gyro_calib_seconds').value)
        self.cmd_period = 1.0 / float(self.get_parameter('cmd_rate_hz').value)
        # Command state
        self.target_pwm = 0.0
        self.command_pwm = 0.0
        self.last_ramp_time = time.time()
        self.servo_us = float(self.steer_center_us)
        self.desired_servo_us = float(self.steer_center_us)
        self.last_servo_update_time = time.time()

        self.last_cmd_send = 0.0
        self.last_sent_pwm = None
        self.last_sent_servo = None

        # Serial state
        self.rx_buffer = ""
        self.arduino_ms = 0
        self.encoder_count_raw = 0
        self.encoder_count = 0.0
        self.last_encoder_count = None
        self.last_encoder_time = None

        self.ax_raw = self.ay_raw = self.az_raw = 0
        self.gx_raw = self.gy_raw = self.gz_raw = 0
        self.arduino_pwm = 0
        self.arduino_servo_us = self.steer_center_us
        self.imu_ok = False

        # Processed state
        self.actual_rpm = 0.0
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.abs_distance_m = 0.0
        self.x_m = 0.0
 self.y_m = 0.0

        self.ax_g = self.ay_g = self.az_g = 0.0
        self.gx_dps = self.gy_dps = self.gz_dps = 0.0
        self.gz_bias = 0.0
        self.gyro_calibrated = False
        self.calib_samples = []
        self.calib_start_time = None
        self.theta_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.last_imu_time = None

        # Scenario state
        self.state = 'WAIT_CALIBRATION'
        self.run_start_time = None
        self.target_heading_deg = 0.0
        self.heading_error_deg = 0.0
        self.steer_correction_us = 0.0
        self.remaining_m = self.target_distance_m
        self.stop_reason = ''

        # ROS publishers
        self.raw_pub = self.create_publisher(String, '/vehicle/state_raw', 10)
        self.scenario_state_pub = self.create_publisher(String, '/scenario1/state', 10)
        self.stop_reason_pub = self.create_publisher(String, '/scenario1/stop_reason', 10)
        self.remaining_pub = self.create_publisher(Float32, '/scenario1/remaining_distance>
        self.heading_error_pub = self.create_publisher(Float32, '/scenario1/heading_error_>
        self.steer_correction_pub = self.create_publisher(Float32, '/scenario1/steer_corre>

        self.imu_ok_pub = self.create_publisher(Bool, '/vehicle/imu_ok', 10)
        self.target_pwm_pub = self.create_publisher(Int32, '/vehicle/target_pwm', 10)
        self.command_pwm_pub = self.create_publisher(Int32, '/vehicle/command_pwm', 10)
        self.applied_pwm_pub = self.create_publisher(Int32, '/vehicle/applied_pwm', 10)
  self.encoder_pub = self.create_publisher(Int32, '/vehicle/encoder_count', 10)
        self.rpm_pub = self.create_publisher(Float32, '/vehicle/actual_rpm', 10)
        self.speed_mps_pub = self.create_publisher(Float32, '/vehicle/speed_mps', 10)
        self.distance_m_pub = self.create_publisher(Float32, '/vehicle/distance_m', 10)
        self.abs_distance_m_pub = self.create_publisher(Float32, '/vehicle/abs_distance_m'>
        self.x_pub = self.create_publisher(Float32, '/vehicle/x_m', 10)
        self.y_pub = self.create_publisher(Float32, '/vehicle/y_m', 10)
        self.theta_pub = self.create_publisher(Float32, '/vehicle/imu_theta_deg', 10)
        self.roll_pub = self.create_publisher(Float32, '/vehicle/imu_roll_deg', 10)
        self.pitch_pub = self.create_publisher(Float32, '/vehicle/imu_pitch_deg', 10)
        self.gz_pub = self.create_publisher(Float32, '/vehicle/imu_gz_dps', 10)
        self.servo_us_pub = self.create_publisher(Int32, '/vehicle/servo_us', 10)
        self.steer_delta_pub = self.create_publisher(Float32, '/vehicle/steering_delta_deg>

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.0, write_timeout=0.02)
            time.sleep(2.0)  # Arduino resets on serial open
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        self.get_logger().info(f'Connected to Arduino on {self.port} @ {self.baud}')
        self.get_logger().info('Keep the vehicle still for IMU gyro calibration.')
        print(HELP)

    # -------------------------
    # Publish helpers
    # -------------------------
    def pub_int(self, pub, value):
        msg = Int32()
        msg.data = int(round(value))
        pub.publish(msg)
    def pub_float(self, pub, value):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def pub_bool(self, pub, value):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def pub_string(self, pub, value):
        msg = String()
        msg.data = str(value)
        pub.publish(msg)

    # -------------------------
    # Serial commands
    # -------------------------
    def send_command(self, force=False):
        now = time.time()
        pwm_i = int(round(self.command_pwm))
        servo_i = int(round(self.servo_us))

        changed = (pwm_i != self.last_sent_pwm) or (servo_i != self.last_sent_servo)
        heartbeat_due = (now - self.last_cmd_send) >= 0.15
        rate_due = (now - self.last_cmd_send) >= self.cmd_period

        if not force and not ((changed and rate_due) or heartbeat_due):
            return

        cmd = f'C,{pwm_i},{servo_i}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.last_cmd_send = now
            self.last_sent_pwm = pwm_i
            self.last_sent_servo = servo_i
  except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def send_text_command(self, text):
        try:
            self.ser.write((text + '\n').encode('utf-8'))
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    # -------------------------
    # Serial read / parse
    # -------------------------
    def read_serial(self):
        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return

            data = self.ser.read(waiting).decode('utf-8', errors='ignore')
            if not data:
                return

            self.rx_buffer += data
            if len(self.rx_buffer) > 2500:
                self.rx_buffer = self.rx_buffer[-700:]
            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                line = line.strip().replace('\r', '')
                if line:
                    self.process_line(line)

        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

    def process_line(self, line):
        self.pub_string(self.raw_pub, line)

        if line.startswith('READY') or line.startswith('A,'):
            return

        m = STATE_RE.match(line)
        if not m:
            return

        (
            t_ms, enc, ax, ay, az, gx, gy, gz,
            applied_pwm, servo_us, imu_ok
        ) = m.groups()

        self.arduino_ms = int(t_ms)
        self.encoder_count_raw = int(enc)
        self.encoder_count = self.encoder_sign * self.encoder_count_raw
        self.ax_raw, self.ay_raw, self.az_raw = int(ax), int(ay), int(az)
        self.gx_raw, self.gy_raw, self.gz_raw = int(gx), int(gy), int(gz)
        self.arduino_pwm = int(applied_pwm)
        self.arduino_servo_us = int(servo_us)
        self.imu_ok = int(imu_ok) == 1
        now = time.time()
        self.process_encoder(now)
        self.process_imu(now)
        self.publish_all()

    # -------------------------
    # Processing on RPi
    # -------------------------
    def process_encoder(self, now):
        count = self.encoder_count

        if self.last_encoder_count is None:
            self.last_encoder_count = count
            self.last_encoder_time = now
            self.distance_m = (count / self.ppr) * self.wheel_circ_m
            self.abs_distance_m = abs(self.distance_m)
            return

        dt = now - self.last_encoder_time
        if dt <= 0.0:
            return

        delta = count - self.last_encoder_count
        raw_rpm = (delta / self.ppr) * (60.0 / dt)
        raw_speed = raw_rpm * self.wheel_circ_m / 60.0

        self.actual_rpm = (self.speed_alpha * raw_rpm) + ((1.0 - self.speed_alpha) * self.>
        self.speed_mps = (self.speed_alpha * raw_speed) + ((1.0 - self.speed_alpha) * self>

        self.distance_m = (count / self.ppr) * self.wheel_circ_m
        self.abs_distance_m = abs(self.distance_m)
        self.remaining_m = max(0.0, self.target_distance_m - self.abs_distance_m)

        if abs(self.speed_mps) < 0.002 and abs(self.command_pwm) < 5:
            self.speed_mps = 0.0
        theta_rad = math.radians(self.theta_deg)
        self.x_m += self.speed_mps * math.cos(theta_rad) * dt
        self.y_m += self.speed_mps * math.sin(theta_rad) * dt

        self.last_encoder_count = count
        self.last_encoder_time = now

    def process_imu(self, now):
        if not self.imu_ok:
            return

        # MPU6050 default scaling: +/-2g and +/-250 deg/s
        self.ax_g = self.ax_raw / 16384.0
        self.ay_g = self.ay_raw / 16384.0
        self.az_g = self.az_raw / 16384.0
        self.gx_dps = self.gx_raw / 131.0
        self.gy_dps = self.gy_raw / 131.0
        raw_gz_dps = self.gz_raw / 131.0

        if not self.gyro_calibrated:
            if self.calib_start_time is None:
                self.calib_start_time = now
                self.calib_samples = []
            self.calib_samples.append(raw_gz_dps)
            if now - self.calib_start_time >= self.gyro_calib_seconds:
                self.gz_bias = sum(self.calib_samples) / max(1, len(self.calib_samples))
                self.gyro_calibrated = True
                self.last_imu_time = now
                self.state = 'READY'
                self.get_logger().info(f'Gyro Z calibrated. Bias = {self.gz_bias:.4f} deg/>
            return

        self.gz_dps = raw_gz_dps - self.gz_bias
        self.roll_deg = math.degrees(math.atan2(self.ay_g, self.az_g))
        self.pitch_deg = math.degrees(math.atan2(-self.ax_g, math.sqrt(self.ay_g * self.ay>

        if self.last_imu_time is None:
            self.last_imu_time = now
    return

        dt = now - self.last_imu_time
        self.last_imu_time = now

        if 0.0 < dt < 0.5:
            self.theta_deg = wrap_angle_deg(self.theta_deg + self.gz_dps * dt)

    def zero_local_encoder_odometry(self):
        self.last_encoder_count = None
        self.last_encoder_time = None
        self.actual_rpm = 0.0
        self.speed_mps = 0.0
        self.distance_m = 0.0
        self.abs_distance_m = 0.0
        self.remaining_m = self.target_distance_m
        self.x_m = 0.0
        self.y_m = 0.0

    def zero_imu_theta(self):
        self.theta_deg = 0.0
        self.last_imu_time = None
        self.target_heading_deg = 0.0

    def steering_delta_deg(self):
        span_us = self.steer_max_us - self.steer_center_us
        if span_us <= 0:
            return 0.0
        return ((self.servo_us - self.steer_center_us) / span_us) * self.steer_max_deg

    # -------------------------
    # Scenario control
    # -------------------------
    def start_scenario(self):
        if not self.gyro_calibrated:
  self.get_logger().warn('Cannot start yet. Keep the car still until gyro calibr>
            return

        self.get_logger().info('Starting Scenario 1: straight-line heading hold to target >
        self.send_text_command('STOP')
        time.sleep(0.05)
        self.send_text_command('ZERO')

        self.zero_local_encoder_odometry()
        self.zero_imu_theta()

        self.target_pwm = 0.0
        self.command_pwm = 0.0
        self.servo_us = float(self.steer_center_us + self.servo_trim_us)
        self.desired_servo_us = self.servo_us
        self.target_heading_deg = 0.0
        self.heading_error_deg = 0.0
        self.remaining_m = self.target_distance_m
        self.stop_reason = ''
        self.run_start_time = time.time()
        self.state = 'RUNNING'
        self.send_command(force=True)

    def stop_scenario(self, reason):
        if self.state != 'DONE':
            self.stop_reason = reason
            self.get_logger().info(f'Scenario stopped: {reason}')
        self.target_pwm = 0.0
        self.command_pwm = 0.0
        self.desired_servo_us = float(self.steer_center_us + self.servo_trim_us)
        self.servo_us = self.desired_servo_us
        self.send_text_command('STOP')
        self.send_command(force=True)
        self.state = 'DONE'
    def emergency_stop_keep_open(self):
        self.stop_reason = 'emergency stop by user'
        self.target_pwm = 0.0
        self.command_pwm = 0.0
        self.desired_servo_us = float(self.steer_center_us + self.servo_trim_us)
        self.servo_us = self.desired_servo_us
        self.send_text_command('STOP')
        self.send_command(force=True)
        self.state = 'READY' if self.gyro_calibrated else 'WAIT_CALIBRATION'
        self.get_logger().warn('Emergency stop. Press g to restart after positioning the c>

    def update_scenario_control(self):
        if self.state != 'RUNNING':
            self.target_pwm = 0.0
            self.command_pwm = 0.0
            self.desired_servo_us = float(self.steer_center_us + self.servo_trim_us)
            self.update_servo_smooth()
            return

        elapsed = time.time() - self.run_start_time if self.run_start_time else 0.0
        self.remaining_m = max(0.0, self.target_distance_m - self.abs_distance_m)

        if self.abs_distance_m >= (self.target_distance_m - self.stop_tolerance_m):
            self.stop_scenario(f'target reached: d={self.abs_distance_m:.3f} m')
            return

        if elapsed > self.max_run_time_s:
            self.stop_scenario(f'timeout: {elapsed:.1f} s')
            return

        if not self.imu_ok:
            self.stop_scenario('IMU lost')
            return

        # Distance-based speed profile.
        if self.remaining_m <= self.final_slow_distance_m:
   desired_mag = self.final_slow_pwm
        elif self.remaining_m <= self.slowdown_distance_m:
            # Linear slowdown from cruise PWM to final slow PWM.
            ratio = (self.remaining_m - self.final_slow_distance_m) / max(0.001, self.slow>
            desired_mag = self.final_slow_pwm + ratio * (self.cruise_pwm - self.final_slow>
        else:
            desired_mag = self.cruise_pwm

        desired_mag = clamp(desired_mag, self.min_move_pwm, self.max_pwm)
        self.target_pwm = desired_mag

        # Heading-hold steering controller.
        self.heading_error_deg = angle_error_deg(self.target_heading_deg, self.theta_deg)
        control_error = self.heading_error_deg
        if abs(control_error) < self.heading_deadband:
            control_error = 0.0

        # Important sign:
        # From your manual run, negative steering helped correct negative theta back towar>
        # Therefore servo correction = -Kp*heading_error + Kd*yaw_rate.
        correction = (-self.heading_kp * control_error) + (self.heading_kd * self.gz_dps)
        correction = clamp(correction, -self.max_steer_correction_us, self.max_steer_corre>
        self.steer_correction_us = correction

        self.desired_servo_us = clamp(
            self.steer_center_us + self.servo_trim_us + correction,
            self.steer_min_us,
            self.steer_max_us
        )
        self.update_servo_smooth()

    def update_pwm_ramp(self):
        now = time.time()
        dt = now - self.last_ramp_time
        self.last_ramp_time = now
        if dt <= 0.0:
   return

        max_step = self.pwm_ramp_per_sec * dt
        if self.command_pwm < self.target_pwm:
            self.command_pwm = min(self.command_pwm + max_step, self.target_pwm)
        elif self.command_pwm > self.target_pwm:
            self.command_pwm = max(self.command_pwm - max_step, self.target_pwm)
        self.command_pwm = clamp(self.command_pwm, -self.max_pwm, self.max_pwm)

    def update_servo_smooth(self):
        now = time.time()
        dt = now - self.last_servo_update_time
        self.last_servo_update_time = now
        if dt <= 0.0:
            return

        max_step = self.servo_rate_us_per_sec * dt
        if self.servo_us < self.desired_servo_us:
            self.servo_us = min(self.servo_us + max_step, self.desired_servo_us)
        elif self.servo_us > self.desired_servo_us:
            self.servo_us = max(self.servo_us - max_step, self.desired_servo_us)
        self.servo_us = clamp(self.servo_us, self.steer_min_us, self.steer_max_us)

    def handle_key(self, key):
        if not key:
            return True

        if key.lower() == 'g':
            if self.state in ['READY', 'DONE']:
                self.start_scenario()
            else:
                self.get_logger().warn(f'Cannot start from state {self.state}')

        elif key.lower() == 's' or key == ' ':
            self.emergency_stop_keep_open()

        elif key.lower() == 'z':
    if self.state != 'RUNNING':
                self.zero_local_encoder_odometry()
                self.send_text_command('ZERO')
                self.get_logger().info('Encoder/distance/x/y zeroed')
            else:
                self.get_logger().warn('Ignored z while running')

        elif key.lower() == 'i':
            if self.state != 'RUNNING':
                self.zero_imu_theta()
                self.get_logger().info('IMU theta zeroed')
            else:
                self.get_logger().warn('Ignored i while running')

        elif key.lower() == 'q':
            self.stop_scenario('quit by user')
            return False

        return True

    def publish_all(self):
        self.pub_string(self.scenario_state_pub, self.state)
        self.pub_string(self.stop_reason_pub, self.stop_reason)
        self.pub_float(self.remaining_pub, self.remaining_m)
        self.pub_float(self.heading_error_pub, self.heading_error_deg)
        self.pub_float(self.steer_correction_pub, self.steer_correction_us)

        self.pub_bool(self.imu_ok_pub, self.imu_ok)
        self.pub_int(self.target_pwm_pub, self.target_pwm)
        self.pub_int(self.command_pwm_pub, self.command_pwm)
        self.pub_int(self.applied_pwm_pub, self.arduino_pwm)
        self.pub_int(self.encoder_pub, self.encoder_count)
        self.pub_float(self.rpm_pub, self.actual_rpm)
        self.pub_float(self.speed_mps_pub, self.speed_mps)
        self.pub_float(self.distance_m_pub, self.distance_m)
        self.pub_float(self.abs_distance_m_pub, self.abs_distance_m)
  self.pub_float(self.x_pub, self.x_m)
        self.pub_float(self.y_pub, self.y_m)
        self.pub_float(self.theta_pub, self.theta_deg)
        self.pub_float(self.roll_pub, self.roll_deg)
        self.pub_float(self.pitch_pub, self.pitch_deg)
        self.pub_float(self.gz_pub, self.gz_dps)
        self.pub_int(self.servo_us_pub, self.servo_us)
        self.pub_float(self.steer_delta_pub, self.steering_delta_deg())

    def print_dashboard(self):
        calib_text = 'OK' if self.gyro_calibrated else 'CAL'
        print(
            f"\rState={self.state:16s} | "
            f"d={self.abs_distance_m:6.3f}/{self.target_distance_m:.2f} m | "
            f"rem={self.remaining_m:5.3f} | "
            f"PWM target/cmd/applied={self.target_pwm:5.1f}/{self.command_pwm:5.1f}/{self.>
            f"v={self.speed_mps:6.3f} m/s | "
            f"theta={self.theta_deg:7.2f} deg | "
            f"err={self.heading_error_deg:6.2f} | "
            f"gz={self.gz_dps:6.2f} | "
            f"servo={self.servo_us:7.1f} us | "
            f"steer={self.steering_delta_deg():6.2f} deg | "
            f"x={self.x_m:6.3f} y={self.y_m:6.3f} | "
            f"IMU={int(self.imu_ok)} {calib_text} | "
            f"g start s stop q quit"
            "\033[K",
            end='',
            flush=True
        )

    def close(self):
        try:
            self.target_pwm = 0.0
            self.command_pwm = 0.0
            self.desired_servo_us = float(self.steer_center_us + self.servo_trim_us)
  self.servo_us = self.desired_servo_us
            self.send_text_command('STOP')
            self.send_command(force=True)
            time.sleep(0.05)
        except Exception:
            pass

        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Scenario1Straight4mNode()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_display = 0.0

    try:
        tty.setraw(fd)
        node.send_text_command('STOP')
        node.send_command(force=True)

        keep_running = True
        while rclpy.ok() and keep_running:
            rclpy.spin_once(node, timeout_sec=0.0)

            node.read_serial()
            key = get_key(timeout=0.001)
            keep_running = node.handle_key(key)
            node.update_scenario_control()
            node.update_pwm_ramp()
            node.send_command()

            now = time.time()
            if now - last_display >= 0.10:
                last_display = now
                node.print_dashboard()

            time.sleep(0.004)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nScenario 1 node stopped safely.')


if __name__ == '__main__':
    main()
