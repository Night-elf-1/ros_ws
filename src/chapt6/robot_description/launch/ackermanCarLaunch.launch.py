#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch
import launch_ros
import xacro


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )
    urdf_path = os.path.join(
        #get_package_share_directory("robot_description"), "urdf", "fourwheel", "base.urdf.xacro"
        get_package_share_directory("robot_description"), "urdf", "ackermanCar", "sim.xacro"
    )
    gazebo_world_path = os.path.join(
        get_package_share_directory("robot_description"), "world", "customtest.world"
    )
    rviz_path = os.path.join(
        get_package_share_directory("robot_description"), "rviz", "robots.rviz"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": xacro.process_file(urdf_path).toxml(),
            }
        ],
    )

    # Gazebo launch
    # Enter below if you encounter this error "shared_ptr assertion error"
    # source /usr/share/gazebo/setup.sh
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"world": gazebo_world_path}.items(),
    )

    gazebo_spawn_entity_node = Node(
        name="urdf_spawner",
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "four_wheel_car"],
        output="screen",
    )

    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path]          # 传递命令行参数
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            gazebo_node,                    # GAZEBO仿真节点
            gazebo_spawn_entity_node,       # Gazebo实体生成节点
            robot_state_publisher_node,     # 发布机器人的状态信息。这里传入我们的刚刚写好的总xacro文件
            rviz_node                       # 启动rviz可视化
        ]
    )
