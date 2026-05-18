ssh pi@AbdElRahman.local



#sceniro 1
cd ~/ros2_ws/src/Autonomous_Systems_Project_Team_19/Autonomous_Systems_Project_Team_19



nano scenario1_straight_4m_team19.py



cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run Autonomous_Systems_Project_Team_19 scenario1_straight_4m_team19 --ros-args \
-p port:=/dev/ttyACM0 \
-p baud:=9600 \
-p target_distance_m:=4.0 \
-p ppr:=1085.0 \
-p cruise_pwm:=170.0 \
-p final_slow_pwm:=85.0 \
-p min_move_pwm:=75.0 \
-p stop_tolerance_m:=0.03 \
-p heading_kp_us_per_deg:=14.0 \
-p heading_kd_us_per_dps:=1.2


--------------------------------------------------------------------
g  start autonomous straight-line scenario
s  emergency stop
z  zero encoder, distance, x, y
i  zero IMU theta
c  center steering
q  stop and quit

-------------------------------------------------------
# sceniro 2


cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run Autonomous_Systems_Project_Team_19 Autonomous_Systems_MS_4_Scenario2_Team_19 --ros-args \
-p port:=/dev/ttyACM0 \
-p baud:=9600 \
-p ppr:=1085.0 \
-p track_end_x_m:=10.0 \
-p lane1_y_m:=0.1875 \
-p lane2_y_m:=-0.1875 \
-p change1_start_x_m:=2.60 \
-p change1_end_x_m:=3.60 \
-p change2_start_x_m:=6.60 \
-p change2_end_x_m:=7.60 \
-p normal_speed_mps:=0.18 \
-p lane_change_speed_mps:=0.11 \
-p max_motor_cmd:=170.0 \
-p k_stanley:=0.75 \
-p max_steer_deg:=23.0 \
-p steer_sign:=1.0 \
-p encoder_sign:=1.0 \
-p theta_sign:=1.0


---------------------------------
1. Put the car at the start line.
2. Keep it straight.
3. Wait until IMU READY.
4. Press z.
5. Press a.

