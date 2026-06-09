"""
view_robot.launch.py

Visualise the DeltaFlex URDF in RViz2 without Gazebo.
Useful for validating the robot model before running a full simulation.

Nodes started:
  - robot_state_publisher  (publishes /robot_description and TF)
  - joint_state_publisher_gui  (manual joint sliders in a GUI window)
  - rviz2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    description_pkg = get_package_share_directory('deltaflex_description')
    xacro_file = os.path.join(description_pkg, 'urdf', 'deltaflex.urdf.xacro')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Pass use_sim:=true to include Gazebo ros2_control tags',
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' use_sim:=', LaunchConfiguration('use_sim'),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
