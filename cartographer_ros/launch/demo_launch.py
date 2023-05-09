import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    map_manager_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('map_manager'), 'test_manager.py')])
    )

    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cartographer_ros'), 'launch', 'test_launch.py')])
    )

    execution_manager_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('flexbotics_execution_manager'),
                'launch', 'flexbotics_execution_manager.launch.xml')]))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
    )

    steering_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='steering',
    )

    return LaunchDescription([
        map_manager_node,
        cartographer_node,
        execution_manager_node,
        rviz_node,
        steering_node
    ])
