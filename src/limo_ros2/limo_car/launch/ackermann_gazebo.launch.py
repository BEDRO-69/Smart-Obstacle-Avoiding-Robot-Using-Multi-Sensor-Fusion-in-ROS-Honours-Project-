import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'limo_car'
    pkg_path = get_package_share_directory(package_name)

    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_path, '..') + ':' + \
        os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_RESOURCE_PATH'] = pkg_path + ':' + \
        os.environ.get('GAZEBO_RESOURCE_PATH', '')

    world_path = os.path.join(pkg_path, 'worlds/open_maze.model')
    default_rviz_config_path = os.path.join(pkg_path, 'rviz/gazebo.rviz')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'ackermann.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'world': world_path}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
   

    )   

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'limo',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5',
                   '-Y', '0.0'],
        output='screen')

    # Delay spawn by 5 seconds to give gzserver time to fully start
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )
    static_tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        delayed_spawn,
        rviz_arg,
        rviz_node,
        static_tf_footprint
    ])
