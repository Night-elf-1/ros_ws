import launch
import launch_ros
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明参数，决定是否生成海龟
    declare_spawn_turtle = launch.actions.DeclareLaunchArgument(
        'spawn_turtle', default_value='False', description='是否生成新海龟'
    )

    spawn_turtle = launch.substitutions.LaunchConfiguration("spawn_turtle")         # 创建spawn_turtle变量，继承参数spawn_turtle的值

    action_turtlesim = launch_ros.actions.Node(         # 创建启动动作
        package="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    # 创建判断条件
    action_executeprocess = launch.actions.ExecuteProcess(
        condition = IfCondition(spawn_turtle),
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{x: 1, y: 1}']        # 如果 IfCondition(spawn_turtle)的条件满足，则执行这个指令
    )

    action_log_info = launch.actions.LogInfo(
        condition = IfCondition(spawn_turtle),
        msg=" 使用 executeprocess 来调用服务生成海龟 "
    )

    # 创建一个动作组，并调用定时器，定时触发launch.actions.LogInfo 和 launch.actions.ExecuteProcess
    action_group = launch.actions.GroupAction(
        [
            launch.actions.TimerAction(period=2.0, actions=[action_log_info]),
            launch.actions.TimerAction(period=3.0, actions=[action_executeprocess]),
        ]
    )

    # 总描述
    launch_description = launch.LaunchDescription(
        [
            declare_spawn_turtle,
            action_turtlesim,
            action_group
        ]
    )