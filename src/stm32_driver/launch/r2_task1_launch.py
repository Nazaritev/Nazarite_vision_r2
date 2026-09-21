import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ====================== odom + cmd_vel fusion for Nav2 open-loop velocity ======================
    odom_cmd_vel_fuser_node = Node(
        package='stm32_driver',
        executable='odom_cmd_vel_fuser',
        name='odom_cmd_vel_fuser',
        output='screen',
        parameters=[{
            'odom_in_topic': '/Odometry',
            'cmd_vel_topic': '/cmd_vel',
            'odom_out_topic': '/nav2_odom',
        }],
    )

    # ====================== nav_agent 导航代理节点 ======================
    nav_agent_node = Node(
        package='stm32_driver',        
        executable='nav2_agent',     
        name='nav_agent',
        output='screen',
        parameters=[{
            'odom_topic': '/nav2_odom',
            'short_distance_controller': 'TEB_Short',
            'long_distance_controller': 'MPPI_Long',
            'controller_switch_distance': 0.50,
            'controller_switch_hysteresis': 0.10,
            'grab_contact_arrival_enabled': True,
            'grab_contact_window_sec': 1.0,
            'grab_contact_position_epsilon': 0.004,
            'grab_contact_max_remaining': 0.04,
            'grab_contact_min_samples': 5,
            'grab_contact_require_preferred_side': True,
            'contact_approach_enabled': True,
            'contact_approach_cmd_vel_topic': '/contact_cmd_vel',
            'contact_approach_speed_x': -0.30,
            'contact_approach_timeout_sec': 8.0,
            'contact_approach_max_distance': 0.60,
            'contact_approach_min_distance': 0.03,
            'contact_approach_command_period_sec': 0.05,
            'contact_approach_require_preferred_side': False,
            'contact_approach_result_delay_sec': 0.3,
            'low_speed_arrival_enabled': True,
            'low_speed_arrival_vx_threshold': 0.05,
            'low_speed_arrival_vy_threshold': 0.05,
            'low_speed_arrival_max_remaining': 0.50,
        }],
    )

    # ====================== r2_mode1 任务流程节点 ======================
    r2_mode1_node = Node(
        package='stm32_driver',          
        executable='r2_mode1',      
        name='r2_mode1',
        output='screen',
        parameters=[{
            'open_loop_grab_enabled': True,
            'pre_grab_open_loop_delay': 0.5,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(odom_cmd_vel_fuser_node)
    ld.add_action(nav_agent_node)
    ld.add_action(r2_mode1_node)

    return ld
