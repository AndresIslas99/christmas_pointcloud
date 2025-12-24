"""
Christmas Point Cloud Launch File
Launches the Christmas cloud publisher and RViz2 for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('christmas_pointcloud')
    
    # RViz config file
    rviz_config = os.path.join(pkg_dir, 'rviz', 'christmas.rviz')
    
    # Declare launch arguments
    tree_height_arg = DeclareLaunchArgument(
        'tree_height',
        default_value='3.0',
        description='Height of the Christmas tree in meters'
    )
    
    num_snowflakes_arg = DeclareLaunchArgument(
        'num_snowflakes',
        default_value='500',
        description='Number of falling snowflakes'
    )
    
    num_ornaments_arg = DeclareLaunchArgument(
        'num_ornaments',
        default_value='50',
        description='Number of ornaments on the tree'
    )
    
    # Christmas cloud publisher node
    christmas_node = Node(
        package='christmas_pointcloud',
        executable='christmas_cloud_node',
        name='christmas_cloud_publisher',
        output='screen',
        parameters=[{
            'tree_height': LaunchConfiguration('tree_height'),
            'tree_base_radius': 1.5,
            'num_tree_points': 15000,
            'num_ornaments': LaunchConfiguration('num_ornaments'),
            'num_snowflakes': LaunchConfiguration('num_snowflakes'),
            'animation_speed': 0.05,
            'snowflake_area': 8.0
        }]
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # Static transform publisher (world frame)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )
    
    return LaunchDescription([
        tree_height_arg,
        num_snowflakes_arg,
        num_ornaments_arg,
        static_tf,
        christmas_node,
        rviz_node
    ])
