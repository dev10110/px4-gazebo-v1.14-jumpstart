#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    PX4_SRC_DIR = '/root/PX4-Autopilot-Quad'
    PX4_BUILD_DIR = os.path.join(PX4_SRC_DIR, '/build/px4_sitl_dasc')

    return LaunchDescription([

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=PX4_SRC_DIR + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=PX4_BUILD_DIR + '/build_gazebo-classic'),
        
        # define a static transform for the camera
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0','0','0','-1.57','0','-1.57', 'px4_1', 'camera_link'],
            parameters = [
                {'use_sim_time': True}
                ],
            ),

        # launch px4's autopilot
        ExecuteProcess(
            cmd=[
                'make','-j', 'px4_sitl_dasc', 'gazebo-classic_visquad__warehouse'
            ],
            cwd=PX4_SRC_DIR,
            output='screen'),

        # launch micrortps_agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'),

])


