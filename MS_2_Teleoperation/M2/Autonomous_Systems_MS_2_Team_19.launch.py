from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    mode = LaunchConfiguration('mode')

    return LaunchDescription([

        # =========================
        # Launch arguments
        # =========================
        DeclareLaunchArgument(
            'linear_speed',
            default_value='1.0'
        ),

        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.5'
        ),

        DeclareLaunchArgument(
            'mode',
            default_value='olr'
        ),

        # =========================
        # Gazebo world
        # =========================
        ExecuteProcess(
            cmd=['gz', 'sim', '/usr/share/gz/gz-sim8/worlds/ackermann_steering.sdf'],
            output='screen'
        ),

        # =========================
        # ROS -> Gazebo bridge for cmd_vel
        # =========================
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/vehicle_blue/cmd_vel]geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),

        # =========================
        # Gazebo -> ROS bridge for odometry
        # =========================
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/vehicle_blue/odometry[gz.msgs.Odometry@nav_msgs/msg/Odometry'],
            output='screen'
        ),

        # =========================
        # OLR node
        # =========================
        Node(
            package='Autonomous_Systems_Project_Team_19',
            executable='Autonomous_Systems_MS_2_OLR_Team_19',
            name='OLR_node',
            output='screen',
            parameters=[
                {'linear_speed': linear_speed},
                {'angular_speed': angular_speed}
            ],
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'olr'"])
            )
        ),

        # =========================
        # Teleop node
        # =========================
        Node(
            package='Autonomous_Systems_Project_Team_19',
            executable='Autonomous_Systems_MS_2_Teleop_Team_19',
            name='Teleop_node',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'teleop'"])
            )
        ),

        # =========================
        # rqt_graph
        # =========================
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen'
        ),
    ])
