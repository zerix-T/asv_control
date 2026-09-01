from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pointcloud_to_laserscan_node = ComposableNode(
        package='asv_control',
        plugin='asv_control::PointcloudToLaserscan',
        name='pointcloud_to_laserscan',

        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/livox/scan'),
        ],

        parameters=[
            {
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.1,

                'min_height': -10.0,
                'max_height': 10.0,

                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,

                'scan_time': 0.1,

                'range_min': 0.1,
                'range_max': 100.0,

                'use_inf': True,
                'inf_epsilon': 1.0,
            }
        ],
    )

    container = ComposableNodeContainer(
        name='pointcloud_to_laserscan_container',
        namespace='',

        package='rclcpp_components',
        executable='component_container',

        composable_node_descriptions=[
            pointcloud_to_laserscan_node
        ],

        output='screen',
    )

    return LaunchDescription([
        container
    ])
