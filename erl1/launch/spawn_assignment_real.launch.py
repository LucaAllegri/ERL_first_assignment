import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_erl1 = get_package_share_directory('erl1')
    pkg_aruco = get_package_share_directory('aruco_opencv')
    aruco_cfg = os.path.join(pkg_aruco, 'config', 'aruco_tracker_real.yaml')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',  
        description='Open RViz.'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='my_robot.urdf',
        description='Name of the URDF description to load'
    )

    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='robot_params.yaml', 
        description='Parameter file for real robot'
    )

    urdf_file_path = PathJoinSubstitution([
        pkg_erl1,
        'urdf',
        LaunchConfiguration('model')
    ])

    params_path = PathJoinSubstitution([
        pkg_erl1,
        'config',
        LaunchConfiguration('params_file')
    ])

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_erl1, 'rviz', 'urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': False},
            {'description_file': urdf_file_path}
        ]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro', ' ', urdf_file_path]),
            'use_sim_time': False
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # ArUco Tracker
    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[
            aruco_cfg,
            {'use_sim_time': False},
        ],
    )

    # Perception Node
    perception_node = Node(
        package='robot_controller',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[
            params_path,
            {'use_sim_time': False}
        ]
    )

    # Motion Node 
    motion_node = Node(
        package='robot_controller',
        executable='robot_motion',
        name='robot_motion',
        output='screen',
        parameters=[
            params_path,
            aruco_cfg,
            {'use_sim_time': False}
        ]
    )

    delayed_motion_node = TimerAction(
        period=5.0, 
        actions=[motion_node]
    )

    ld = LaunchDescription()

    ld.add_action(rviz_launch_arg)
    ld.add_action(model_arg)
    ld.add_action(config_file_arg)

    ld.add_action(rviz_node)
    #ld.add_action(robot_state_publisher_node)
    ld.add_action(aruco_node)
    ld.add_action(perception_node)
    ld.add_action(delayed_motion_node)

    return ld