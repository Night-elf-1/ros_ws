# import os
# import launch
# import launch_ros
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#     # 获取与拼接默认路径
#     fishbot_navigation2_dir = get_package_share_directory(
#         'robot_description')
#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
#     rviz_config_dir = os.path.join(
#         nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')                         # 获取nav2_bringup下面的rviz2配置文件
    
#     # 创建 Launch 配置
#     use_sim_time = launch.substitutions.LaunchConfiguration(
#         'use_sim_time', default='true')
#     map_yaml_path = launch.substitutions.LaunchConfiguration(
#         'map', default=os.path.join(fishbot_navigation2_dir, 'map', 'map_2024-11-29_11-49-04.yaml'))            # 获取建好的地图
#     nav2_param_path = launch.substitutions.LaunchConfiguration(
#         'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

#     return launch.LaunchDescription([
#         # 声明新的 Launch 参数
#         launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
#                                              description='Use simulation (Gazebo) clock if true'),
#         launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,                                
#                                              description='Full path to map file to load'),
#         launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
#                                              description='Full path to param file to load'),

#         launch.actions.IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
#             # 使用 Launch 参数替换原有参数
#             launch_arguments={
#                 'map': map_yaml_path,                                       # 将地图路径传给bringup
#                 'use_sim_time': use_sim_time,
#                 'params_file': nav2_param_path}.items(),
#         ),
#         launch_ros.actions.Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             parameters=[{'use_sim_time': use_sim_time}],
#             output='screen'),
#     ])



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('robot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(fishbot_navigation2_dir,'map','map_2024-11-29_11-49-04.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(fishbot_navigation2_dir,'config','nav2_params.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    return LaunchDescription([nav2_bringup_launch,rviz_node])
