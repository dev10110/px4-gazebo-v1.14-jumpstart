#!/usr/bin/env python3
import math
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

    # GAZEBO_MODEL_PATH = f"{PX4_SRC_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:/root/colcon_ws/src/forest_gen/models_low_res:/root/colcon_ws/src/forest_gen/models_high_res"
    # GAZEBO_MODEL_PATH = "/root/colcon_ws/src/forest_gen/models_low_res"

    # print(GAZEBO_MODEL_PATH)
    

    return LaunchDescription([

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=PX4_SRC_DIR + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'),
        # SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=GAZEBO_MODEL_PATH),
        # SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value="${GAZBO_PLUGIN_PATH}:" + PX4_BUILD_DIR + '/build_gazebo-classic'),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=PX4_BUILD_DIR + '/build_gazebo-classic'),
        SetEnvironmentVariable(name="PX4_NO_FOLLOW_MODE", value="1"),
        # SetEnvironmentVariable(name="HEADLESS", value="1"),
        SetEnvironmentVariable(name="PX4_SITL_WORLD", value="/root/colcon_ws/src/forest_gen/easy_worlds/forest0.world"), # give it an absolute path to the world
        SetEnvironmentVariable(name="VERBOSE_SIM", value="1"),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', '${GAZEBO_RESOURCE_PATH}:/usr/share/gazebo-11:/root/colcon_ws/src/forest_gen'),

        SetEnvironmentVariable("PX4_SPAWN_LOCATION_X", value="-27.0"),
        SetEnvironmentVariable("PX4_SPAWN_LOCATION_Y",  value="0.0"),
        SetEnvironmentVariable("PX4_SPAWN_LOCATION_Z",  value="1.0"),
    
        # define a static transform for the camera
        Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=['0','0','0','-1.57','0','-1.57', 'vicon/px4_1/px4_1', 'camera_link'],
                parameters = [
                    {'use_sim_time': True}
                    ],
                ),
        
        # define vicon/world/NED"
        Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0","0","0", f"{math.pi/2}", "0", f"{math.pi}", "/vicon/world/NED", "/vicon/world"
                    ]
                ),
            
        # launch px4's autopilot
        ExecuteProcess(
                 cmd=[
                     # 'make','-j', 'px4_sitl_dasc', 'gazebo-classic_visquad__warehouse'
                     'make', '-j', 'px4_sitl_dasc', 'gazebo-classic_visquad'
                 ],
                 cwd=PX4_SRC_DIR,
                 output='screen'),
            
        # launch micrortps_agent
        ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                output='screen'),
        
        # gazebo-px4-bridge
        Node(
                package="gazebo_px4_bridge",
                executable="gazebo_px4_bridge",
                parameters = [
                    {'use_sim_time': True}
                    ]
                ),

        # rviz
        ExecuteProcess(
                cmd=[
                    'rviz2','-d', 'default.rviz',
                ],
                cwd="/root/colcon_ws/src/sim_launch/config",
                output='screen'),
        
        ]
        )


