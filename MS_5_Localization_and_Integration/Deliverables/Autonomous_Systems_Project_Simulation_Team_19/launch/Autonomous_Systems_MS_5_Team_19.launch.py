from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'autonomous_systems_project_team_19'

    track = LaunchConfiguration('track')
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    launch_rqt_graph = LaunchConfiguration('launch_rqt_graph')
    max_speed = LaunchConfiguration('max_speed')
    lane_change_speed = LaunchConfiguration('lane_change_speed')
    corner_speed = LaunchConfiguration('corner_speed')
    steering_sign = LaunchConfiguration('steering_sign')
    city_no_steering_until_x = LaunchConfiguration('city_no_steering_until_x')

    world_file = PythonExpression([
        "'Empty_Track.world' if '", track, "' == '1' else ",
        "('Racing_Track.world' if '", track, "' == '2' else 'City_Track.world')"
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'Worlds',
        world_file,
    ])

    ideal_odom_topic = '/model/vehicle_blue/odometry'
    noisy_odom_topic = '/ms5/noisy_odom'
    filtered_odom_topic = '/ms5/filtered_odom'
    cmd_vel_topic = '/model/vehicle_blue/cmd_vel'

    return LaunchDescription([
        DeclareLaunchArgument('track', default_value='2', description='1 = Empty Track, 2 = Racing Track, 3 = City Track'),
        DeclareLaunchArgument('launch_gazebo', default_value='true'),
        DeclareLaunchArgument('launch_rqt_graph', default_value='true'),
        DeclareLaunchArgument('max_speed', default_value='0.12'),
        DeclareLaunchArgument('lane_change_speed', default_value='0.07'),
        DeclareLaunchArgument('corner_speed', default_value='0.025'),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument('city_no_steering_until_x', default_value='1.55'),
        DeclareLaunchArgument('x_noise_std', default_value='0.015'),
        DeclareLaunchArgument('y_noise_std', default_value='0.015'),
        DeclareLaunchArgument('theta_noise_std', default_value='0.008'),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen',
            condition=IfCondition(launch_gazebo),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ms5_ros_gz_bridge_team19',
            output='screen',
            arguments=[
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            ],
        ),

        Node(
            package=package_name,
            executable='Autonomous_Systems_MS_5_Localization_Team_19',
            name='Autonomous_Systems_MS_5_Localization_Team_19',
            output='screen',
            parameters=[{
                'ideal_odom_topic': ideal_odom_topic,
                'noisy_odom_topic': noisy_odom_topic,
                'filtered_odom_topic': filtered_odom_topic,
                'x_noise_std': LaunchConfiguration('x_noise_std'),
                'y_noise_std': LaunchConfiguration('y_noise_std'),
                'theta_noise_std': LaunchConfiguration('theta_noise_std'),
                'v_noise_std': 0.015,
                'omega_noise_std': 0.015,
                'process_noise_q': 0.002,
                'measurement_noise_r': 0.035,
            }],
        ),

        Node(
            package=package_name,
            executable='Autonomous_Systems_MS_5_Planning_Team_19',
            name='Autonomous_Systems_MS_5_Planning_Team_19',
            output='screen',
            parameters=[{
                'track_id': track,
                'odom_topic': filtered_odom_topic,
                'track_length': 10.0,
                'finish_x': 9.70,
                'lane1_y': 0.1875,
                'lane2_y': -0.1875,
                'straight_y': 0.0,
                'max_speed': max_speed,
                'lane_change_speed': lane_change_speed,
                'corner_speed': corner_speed,
                'city_lookahead_index': 10,
                'publish_rate': 40.0,
            }],
        ),

        Node(
            package=package_name,
            executable='Autonomous_Systems_MS_5_Speed_Controller_Team_19',
            name='Autonomous_Systems_MS_5_Speed_Controller_Team_19',
            output='screen',
            parameters=[{
                'odom_topic': filtered_odom_topic,
                'desired_speed_topic': '/ms4/desired_speed',
                'speed_cmd_topic': '/ms3/speed_cmd',
                'max_speed_cmd': 0.45,
                'finish_x': 999.0,
                'control_rate': 40.0,
            }],
        ),

        Node(
            package=package_name,
            executable='Autonomous_Systems_MS_5_Stanley_Lateral_Team_19',
            name='Autonomous_Systems_MS_5_Stanley_Lateral_Team_19',
            output='screen',
            parameters=[{
                'track_id': track,
                'odom_topic': filtered_odom_topic,
                'desired_lane_topic': '/ms4/desired_lane_y',
                'desired_yaw_topic': '/ms4/desired_yaw',
                'desired_cte_topic': '/ms4/desired_cte',
                'steering_lock_topic': '/ms4/steering_lock',
                'steering_cmd_topic': '/ms3/steering_cmd',
                'stanley_gain': 0.80,
                'city_stanley_gain': 0.70,
                'softening_gain': 0.45,
                'heading_gain': 1.6,
                'city_heading_gain': 1.00,
                'max_steering_angle': 0.46,
                'minimum_speed_for_stanley': 0.05,
                'steering_sign': steering_sign,
                'steering_filter_alpha': 0.45,
                'max_steering_rate': 0.90,
                'control_rate': 40.0,
            }],
        ),

        Node(
            package=package_name,
            executable='Autonomous_Systems_MS_5_Gazebo_Command_Mux_Team_19',
            name='Autonomous_Systems_MS_5_Gazebo_Command_Mux_Team_19',
            output='screen',
            parameters=[{
                'speed_cmd_topic': '/ms3/speed_cmd',
                'steering_cmd_topic': '/ms3/steering_cmd',
                'cmd_vel_topic': cmd_vel_topic,
                'wheel_base': 0.22,
                'max_linear_speed': 0.45,
                'max_steering_angle': 0.46,
                'max_yaw_rate': 1.20,
                'publish_rate': 40.0,
                'command_timeout': 1.0,
            }],
        ),

        Node(
            package=package_name,
            executable='Autonomous_Systems_MS_5_Graph_Recorder_Team_19',
            name='Autonomous_Systems_MS_5_Graph_Recorder_Team_19',
            output='screen',
            parameters=[{
                'track_id': track,
                'ideal_odom_topic': ideal_odom_topic,
                'noisy_odom_topic': noisy_odom_topic,
                'filtered_odom_topic': filtered_odom_topic,
                'desired_speed_topic': '/ms4/desired_speed',
                'desired_lane_topic': '/ms4/desired_lane_y',
                'steering_cmd_topic': '/ms3/steering_cmd',
                'speed_cmd_topic': '/ms3/speed_cmd',
                'sample_rate': 20.0,
            }],
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['rqt_graph'],
                    output='screen',
                    condition=IfCondition(launch_rqt_graph),
                )
            ],
        ),
    ])
