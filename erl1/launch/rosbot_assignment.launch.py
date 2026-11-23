import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_aruco = get_package_share_directory('aruco_opencv')
    aruco_cfg = os.path.join(pkg_aruco, 'config', 'aruco_tracker.yaml')
    # Gazebo camera in my_robot.gazebo, 640 px wide with an 80-degree horizontal FOV.
    sim_fx = 640.0 / (2.0 * math.tan(1.3962634 / 2.0))
    camera_topics = [
        ('/camera/image', LaunchConfiguration('image_topic')),
        ('/camera/camera_info', LaunchConfiguration('camera_info_topic')),
    ]
    marker_side = ParameterValue(LaunchConfiguration('marker_side'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('reference_fx_px', default_value=str(sim_fx)),
        DeclareLaunchArgument('marker_side', default_value='0.2'),

        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            output='screen',
            parameters=[aruco_cfg, {'cam_base_topic': '/camera/image', 'marker_size': marker_side}],
            remappings=camera_topics,
        ),
        Node(
            package='robot_controller',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'marker_side': marker_side,
                'target_side_px': 166.0,
                'reference_fx_px': ParameterValue(LaunchConfiguration('reference_fx_px'), value_type=float),
            }],
            remappings=camera_topics,
        ),
        Node(
            package='robot_controller',
            executable='robot_motion',
            name='robot_motion',
            output='screen',
            parameters=[{'k_v': 0.005, 'k_w': 0.001, 'k_a': 0.3}],
            remappings=[
                ('/odom', LaunchConfiguration('odom_topic')),
                ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ],
        ),
    ])
