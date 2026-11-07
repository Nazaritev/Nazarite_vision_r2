import launch
import launch_ros

def generate_launch_description():
    action1=launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'speed': 0.4,   # 线速度初始值
            'turn': 3.14,   # 角速度初始值
        }],
    )
    action2= launch_ros.actions.Node(
        package='run_keyboard_test',
        executable='get_and_pub',
        output='screen'
    )
    # action3= launch_ros.actions.Node(
    #     package='demo_python_work2_2',
    #     executable='KillAndBuild',
    #     output='screen'
    # )
    action = launch.actions.GroupAction([
        # 动作5-定时器
        launch.actions.TimerAction(period=0.0, actions=[action1]), #0秒后执行
        launch.actions.TimerAction(period=0.0, actions=[action2]), #0秒后执行
        # launch.actions.TimerAction(period=0.0, actions=[action3]), #4秒后执行

    ])
    return launch.LaunchDescription([
        action,
    ])
# ros2 launch run_keyboard_test teleop_run.launch.py