from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='mapper',
            namespace='mapper',
            output='screen',
            remappings=[
                ('/mapper/scan', '/scan'),
                ('/mapper/odom', '/odom')
            ],
            arguments=[
                '-configuration_directory', '/root/workspaces/boeing_sim/src/bmpr_core/bmpr_core_bringup/config/cartographer'
            ],
            parameters=[{'use_sim_time': True}]
        )
    ])