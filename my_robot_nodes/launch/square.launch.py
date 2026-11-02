from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import TimerAction

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # Launch argument for the square side length
    side_length_arg = DeclareLaunchArgument(
        'side_length',
        default_value='0.5',
        description='Length of each side of the square in meters'
    )

    # Locate diffdrive_bot package
    diffdrive_share = get_package_share_directory(
        'diffdrive_bot'
    )

    # Include diffdrive_bot control launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                diffdrive_share,
                'launch',
                'control.launch.py'
            )
        )
    )

    # Start the rotation action server
    rotate_server = Node(
        package='my_robot_nodes',
        executable='rotate_server',
        name='rotate_server',
        output='screen'
    )

    # Start the square-driving action client
    # Start the square driver after Gazebo and RViz have had time to initialize
    square_driver = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='my_robot_nodes',
                executable='square_driver',
                name='square_driver',
                output='screen',
                parameters=[
                    {
                        'side_length': LaunchConfiguration('side_length')
                    }
                ]
            )
        ]
    )

    return LaunchDescription([
        side_length_arg,
        control_launch,
        rotate_server,
        square_driver
    ])