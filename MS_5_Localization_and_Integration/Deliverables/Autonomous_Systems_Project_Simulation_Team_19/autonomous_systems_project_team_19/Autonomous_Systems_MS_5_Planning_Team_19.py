#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String

from .utils import quaternion_from_yaw, yaw_from_quaternion, normalize_angle, clamp


class MS5Planner(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Planning_Team_19')

        self.declare_parameter('track_id', 2)  # 1 Empty, 2 Racing, 3 City
        self.declare_parameter('odom_topic', '/ms5/filtered_odom')
        self.declare_parameter('track_length', 10.0)
        self.declare_parameter('finish_x', 9.70)
        self.declare_parameter('track3_finish_x', 72.0)
        self.declare_parameter('lane1_y', 0.1875)
        self.declare_parameter('lane2_y', -0.1875)
        self.declare_parameter('straight_y', 0.0)
        self.declare_parameter('max_speed', 0.14)
        self.declare_parameter('lane_change_speed', 0.07)
        self.declare_parameter('corner_speed', 0.030)
        self.declare_parameter('publish_rate', 40.0)

        self.track_id = int(self.get_parameter('track_id').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.track_length = float(self.get_parameter('track_length').value)
        self.finish_x = float(self.get_parameter('finish_x').value)
        self.track3_finish_x = float(self.get_parameter('track3_finish_x').value)
        self.lane1_y = float(self.get_parameter('lane1_y').value)
        self.lane2_y = float(self.get_parameter('lane2_y').value)
        self.straight_y = float(self.get_parameter('straight_y').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.lane_change_speed = float(self.get_parameter('lane_change_speed').value)
        self.corner_speed = float(self.get_parameter('corner_speed').value)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False

        # Track 3 uses the same distance-based planning logic as the MS4 planner you provided.
        self.distance_traveled = 0.0
        self.prev_x = None
        self.prev_y = None

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.pub_speed = self.create_publisher(Float64, '/ms4/desired_speed', 20)
        self.pub_lane_y = self.create_publisher(Float64, '/ms4/desired_lane_y', 20)
        self.pub_yaw = self.create_publisher(Float64, '/ms4/desired_yaw', 20)
        self.pub_cte = self.create_publisher(Float64, '/ms4/desired_cte', 20)
        self.pub_steering_lock = self.create_publisher(Float64, '/ms4/steering_lock', 20)
        self.pub_state = self.create_publisher(String, '/ms4/planner_state', 20)
        self.pub_path = self.create_publisher(Path, '/ms4/desired_path', 10)

        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_plan)

        if self.track_id == 1:
            self.get_logger().info('MS5 planner: Track 1 longitudinal-only lane keeping at y=0')
        elif self.track_id == 2:
            self.get_logger().info('MS5 planner: Track 2 lane-change scenario')
        else:
            self.get_logger().info('MS5 planner: Track 3 PHASED city lap. No early corner steering.')

    @staticmethod
    def smoothstep(t: float) -> float:
        t = clamp(t, 0.0, 1.0)
        return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)

    @staticmethod
    def smoothstep_derivative(t: float) -> float:
        t = clamp(t, 0.0, 1.0)
        return 30.0 * t * t * (1.0 - t) * (1.0 - t)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.prev_x is None:
            self.prev_x = self.current_x
            self.prev_y = self.current_y
            self.odom_ready = True
            return

        dx = self.current_x - self.prev_x
        dy = self.current_y - self.prev_y
        step_distance = math.sqrt(dx * dx + dy * dy)

        # Avoid adding unrealistically large odometry jumps caused by reset/glitch.
        if step_distance < 0.20:
            self.distance_traveled += step_distance

        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.odom_ready = True

    def track1_reference(self, x: float):
        return self.straight_y, 0.0, False, 'TRACK_1_KEEP_CENTER'

    def track2_reference(self, x: float):
        x = clamp(x, 0.0, self.track_length)
        lane1 = self.lane1_y
        lane2 = self.lane2_y
        if x < 1.40:
            y = lane1; dydx = 0.0; state = 'KEEP_LANE_1'
        elif x < 3.45:
            x0, x1 = 1.40, 3.45
            t = (x - x0) / (x1 - x0)
            s = self.smoothstep(t)
            dsdt = self.smoothstep_derivative(t)
            y = lane1 + s * (lane2 - lane1)
            dydx = (lane2 - lane1) * dsdt / (x1 - x0)
            state = 'LANE_CHANGE_TO_LANE_2'
        elif x < 5.55:
            y = lane2; dydx = 0.0; state = 'KEEP_LANE_2'
        elif x < 7.65:
            x0, x1 = 5.55, 7.65
            t = (x - x0) / (x1 - x0)
            s = self.smoothstep(t)
            dsdt = self.smoothstep_derivative(t)
            y = lane2 + s * (lane1 - lane2)
            dydx = (lane1 - lane2) * dsdt / (x1 - x0)
            state = 'LANE_CHANGE_TO_LANE_1'
        else:
            y = lane1; dydx = 0.0; state = 'KEEP_LANE_1'
        yaw = math.atan2(dydx, 1.0)
        return y, yaw, 'LANE_CHANGE' in state, state

    def reference_at_x(self, x: float):
        if self.track_id == 1:
            return self.track1_reference(x)
        return self.track2_reference(x)

    def clamp_track3_y(self, y):
        return max(-3.0, min(3.0, y))
    def quintic_smoothstep(self, t: float) -> float:
        t = max(0.0, min(1.0, t))
        return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)
    def reference_track3(self, x: float):
        """
        Track 3 city track.
        x = distance_traveled (accumulated odometry distance around the track).

        Road geometry from City_Track.world:
        bottom_road: y_center = -2.02,  driving +x, right lane y = -2.2075
        right_road:  x_center =  2.52,  driving +y, right lane x =  2.3325
        top_road:    y_center =  2.02,  driving -x, right lane y =  2.2075
        left_road:   x_center = -2.52,  driving -y, right lane x = -2.3325

        Estimated cumulative distance thresholds:
        0.0  -> 6.0   : bottom straight
        6.0  -> 6.8   : bottom-right corner
        6.8  -> 11.8  : right straight
        11.8 -> 12.6  : top-right corner
        12.6 -> 18.6  : top straight
        18.6 -> 19.4  : top-left corner
        19.4 -> 24.4  : left straight
        24.4 -> 25.2  : bottom-left corner
        25.2+         : lap complete / second lap bottom straight
        """

        #BOTTOM_Y = 0.0   # right lane on bottom road (going +x)
        #RIGHT_Y  =  9.4   # right lane on right road  (going +y)
        #TOP_Y    =  0.0   # right lane on top road    (going -x)
        #LEFT_Y   = -9.4   # right lane on left road   (going -y)
        BOTTOM_Y = self.current_y
        RIGHT_Y  = self.current_y
        TOP_Y    = self.current_y
        LEFT_Y   = self.current_y
        # ── Bottom straight ──────────────────────────────────────────
        if x < 23.2:
            desired_y   = BOTTOM_Y
            desired_yaw = 0.0
            state = "CITY_STRAIGHT_BOTTOM"

        # ── Bottom-right corner (turn from +x to +y) ─────────────────
        elif x < 30.0:
            t = (x - 23.1) / 6.8
            t = self.quintic_smoothstep(t)
            desired_y   = BOTTOM_Y + t * (RIGHT_Y - BOTTOM_Y)
            desired_yaw = t * math.pi / 2.0
            state = "CITY_TURN_BOTTOM_RIGHT"

        # ── Right straight ────────────────────────────────────────────
        elif x < 34.0:
            desired_y   = RIGHT_Y
            desired_yaw = math.pi / 2.0       # facing +y
            state = "CITY_STRAIGHT_RIGHT"

        # ── Top-right corner (turn from +y to -x) ────────────────────
        elif x < 40.3:
            t = (x - 34.0) / 6.3
            t = self.quintic_smoothstep(t)
            desired_y   = RIGHT_Y + t * (TOP_Y - RIGHT_Y)
            desired_yaw = math.pi / 2.0 + t * math.pi / 2.0
            state = "CITY_TURN_TOP_RIGHT"

        # ── Top straight ──────────────────────────────────────────────
        elif x < 52.5:
            desired_y   = TOP_Y
            desired_yaw = math.pi             # facing -x
            state = "CITY_STRAIGHT_TOP"

        # ── Top-left corner (turn from -x to -y) ─────────────────────
        elif x < 62.8:
            t = (x - 60.1) / 2.7
            t = self.quintic_smoothstep(t)
            desired_y   = TOP_Y + t * (LEFT_Y - TOP_Y)
            desired_yaw = math.pi + t * math.pi / 2.0
            state = "CITY_TURN_TOP_LEFT"

        # ── Left straight ─────────────────────────────────────────────
        elif x < 78.7:
            desired_y   = LEFT_Y
            desired_yaw = 3.0 * math.pi / 2.0  # facing -y (= 4.7124 rad)
            state = "CITY_STRAIGHT_LEFT_DOWN"

        # ── Bottom-left corner (turn from -y back to +x) ─────────────
        elif x < 81.5:
            t = (x - 78.7) / 2.8
            t = self.quintic_smoothstep(t)
            desired_y   = LEFT_Y + t * (BOTTOM_Y - LEFT_Y)
            desired_yaw = 3.0 * math.pi / 2.0 + t * math.pi / 2.0
            state = "CITY_TURN_BOTTOM_LEFT"

        # ── Lap complete ──────────────────────────────────────────────
        else:
            desired_y   = BOTTOM_Y
            desired_yaw = 0.0
            state = "CITY_LAP_COMPLETE"

        is_slow_region = "TURN" in state
        finish_x = self.track3_finish_x

        return desired_y, desired_yaw, is_slow_region, state, finish_x

    def publish_reference_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        if self.track_id == 3:
            x = 0.0
            while x <= self.track3_finish_x:
                y, yaw, _, _, _ = self.reference_track3(x)
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                pose.pose.orientation = quaternion_from_yaw(yaw)
                path.poses.append(pose)
                x += 0.10
        else:
            x = 0.0
            while x <= self.track_length:
                y, yaw, _, _ = self.reference_at_x(x)
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0
                pose.pose.orientation = quaternion_from_yaw(yaw)
                path.poses.append(pose)
                x += 0.10
        self.pub_path.publish(path)

    def publish_plan(self):
        if self.track_id == 3:
            desired_y, desired_yaw, is_slow_region, state_name, finish_x = self.reference_track3(self.distance_traveled)

            if not self.odom_ready:
                speed = 0.0
                state = 'MAP_3_WAITING_FOR_FILTERED_ODOMETRY'
            elif self.distance_traveled >= self.track3_finish_x:
                speed = 0.0
                state = 'MAP_3_FINISH_STOP'
            else:
                speed = self.corner_speed if is_slow_region else self.max_speed
                state = f'MAP_3_{state_name}'

            self.pub_speed.publish(Float64(data=float(speed)))
            self.pub_lane_y.publish(Float64(data=float(desired_y)))
            self.pub_yaw.publish(Float64(data=float(desired_yaw)))
            self.pub_cte.publish(Float64(data=0.0))
            self.pub_steering_lock.publish(Float64(data=0.0))
            msg = String()
            msg.data = (f'{state} | track=3 | x={self.current_x:.2f} | y={self.current_y:.3f} '
                        f'| distance={self.distance_traveled:.2f} | desired_y={desired_y:.3f} '
                        f'| desired_yaw={desired_yaw:.3f} | speed={speed:.2f}')
            self.pub_state.publish(msg)
            self.publish_reference_path()
            self.get_logger().info(msg.data, throttle_duration_sec=0.5)
            return

        desired_y, desired_yaw, lane_change, state_name = self.reference_at_x(self.current_x)
        if not self.odom_ready:
            speed = 0.0; state = 'WAITING_FOR_FILTERED_ODOMETRY'
        elif self.current_x >= self.finish_x:
            speed = 0.0; state = 'FINISH_STOP'
        else:
            speed = self.lane_change_speed if lane_change else self.max_speed
            state = state_name
        self.pub_speed.publish(Float64(data=float(speed)))
        self.pub_lane_y.publish(Float64(data=float(desired_y)))
        self.pub_yaw.publish(Float64(data=float(desired_yaw)))
        self.pub_cte.publish(Float64(data=0.0))
        self.pub_steering_lock.publish(Float64(data=0.0))
        msg = String()
        msg.data = f'{state} | track={self.track_id} | x={self.current_x:.2f} | y={self.current_y:.3f} | desired_y={desired_y:.3f} | desired_yaw={desired_yaw:.3f} | speed={speed:.2f}'
        self.pub_state.publish(msg)
        self.publish_reference_path()
        self.get_logger().info(msg.data, throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MS5Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
