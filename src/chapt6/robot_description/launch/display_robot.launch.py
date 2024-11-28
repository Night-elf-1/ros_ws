import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory             # 找包库


def generate_launch_description():
    # 获取默认路径
    urdf_tutorial_path = get_package_share_directory('robot_description')           # 获取包路径 注意这里找的是install下的功能包（编译后的）
    default_model_path = urdf_tutorial_path + '/urdf/b_class_car/base.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path + '/rviz/robots.rviz'
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(                                           # 将xacro文件转换成urdf文件命令 xacro + 文件路径
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(                   # 发布agv的状态 运行这个功能包robot_state_publisher
        package='robot_state_publisher',
        executable='robot_state_publisher',                                 # 该功能包下的可执行文件
        parameters=[{'robot_description': robot_description}]               # 通过robot_description 传递xacro文件的内容给robot_state_publisher
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]          # 传递命令行参数
    )
    return launch.LaunchDescription([                       # 返回一个launch描述
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])