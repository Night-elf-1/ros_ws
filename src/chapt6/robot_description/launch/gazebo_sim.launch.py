import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = "b_class_car"
    urdf_tutorial_path = get_package_share_directory('robot_description')
    default_model_path = urdf_tutorial_path + '/urdf/b_class_car/base.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/custom.world'
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
  	
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(         
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),                    # gazebo_ros功能包提供启动gazebo的launch文件 gazebo.launch.py
      	# 传递参数
        launch_arguments=[('world', default_world_path),('verbose','true')]     # verbose为是否显示详细日志 这里设置为true
    )
    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(                # 创建spawn_entity_node 调用gazebo_ros功能包下的spawn_entity.py节点 会从/robot_description话题获取
        package='gazebo_ros',                                   # 机器人模型
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, ])
    
    # # 加载并激活 fishbot_joint_state_broadcaster 控制器
    # load_joint_state_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'fishbot_joint_state_broadcaster'],
    #     output='screen'
    # )

    # # 加载并激活 fishbot_effort_controller 控制器
    # load_fishbot_effort_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
    #     output='screen')
    
    # load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
    #     output='screen')
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        # # 事件动作，当加载机器人结束后执行    
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=spawn_entity_node,
        #         on_exit=[load_joint_state_controller],)
        #     ),
        # # 事件动作，load_fishbot_diff_drive_controller
        # launch.actions.RegisterEventHandler(
        # event_handler=launch.event_handlers.OnProcessExit(
        #     target_action=load_joint_state_controller,
        #     on_exit=[load_fishbot_diff_drive_controller],)
        #     ),
    ])





# import launch
# import launch_ros
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     # 获取默认路径
#     robot_name_in_model = "b_class_car"
#     urdf_tutorial_path = get_package_share_directory('robot_description')
#     default_model_path = urdf_tutorial_path + '/urdf/b_class_car/base.urdf.xacro'
#     default_world_path = urdf_tutorial_path + '/world/custom.world'
#     control_config_path = urdf_tutorial_path + '/config/controllers.yaml'  # 控制器配置文件路径

#     # 声明参数
#     action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
#         name='model', default_value=str(default_model_path),
#         description='URDF 的绝对路径')
#     robot_description = launch_ros.parameter_descriptions.ParameterValue(
#         launch.substitutions.Command(
#             ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
#         value_type=str)
    
#     # robot_state_publisher 节点
#     robot_state_publisher_node = launch_ros.actions.Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_description}]
#     )

#     # Gazebo 启动
#     launch_gazebo = launch.actions.IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             get_package_share_directory('gazebo_ros'),
#             '/launch/gazebo.launch.py'
#         ]),
#         launch_arguments=[('world', default_world_path), ('verbose', 'true')]
#     )

#     # 机器人加载
#     spawn_entity_node = launch_ros.actions.Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', '/robot_description', '-entity', robot_name_in_model]
#     )
    
#     # 加载控制器
#     load_joint_state_controller = launch.actions.ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
#         output='screen'
#     )
#     load_diff_drive_controller = launch.actions.ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
#         output='screen'
#     )
    
#     # LaunchDescription
#     return launch.LaunchDescription([
#         action_declare_arg_mode_path,
#         robot_state_publisher_node,
#         launch_gazebo,
#         spawn_entity_node,
#         # 确保在实体加载完成后加载控制器
#         launch.actions.RegisterEventHandler(
#             launch.event_handlers.OnProcessExit(
#                 target_action=spawn_entity_node,
#                 on_exit=[load_joint_state_controller, load_diff_drive_controller]
#             )
#         )
#     ])
