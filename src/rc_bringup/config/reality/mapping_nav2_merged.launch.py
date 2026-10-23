import atexit
import os
import tempfile

import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

NAV2_PARAMS_YAML = r"""
lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names: ['controller_server',
                 'planner_server',
                 'behavior_server',
                 'bt_navigator',
                 'global_costmap/global_costmap',
                 'local_costmap/local_costmap']

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: odom
    robot_base_frame: footprint_link
    odom_topic: /nav2_odom
    bt_loop_duration: 10
    default_nav_to_pose_bt_xml: "/home/dgut/code/Nazarite/src/rc_bringup/config/reality/navigate_to_pose_teb_mppi_selector.xml"
    default_nav_through_poses_bt_xml: "/home/dgut/code/Nazarite/src/rc_bringup/config/reality/nav2_mult.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_navigate_through_poses_action_bt_node

#planner_server:
#  ros__parameters:
#    use_sim_time: false
#    planner_plugins: ["GridBased"]
#    GridBased:
#      plugin: "nav2_smac_planner/SmacPlanner2D"
#      tolerance: 0.00
#      max_iterations: 500000
#      max_on_approach_iterations: 500
#      allow_unknown: true
#      use_final_approach_orientation: false
#      motion_model: "OMNI"
#      radius: 0.20
#      analytical_expansion: false
#      analytical_expansion_max_length: 1.5
#      min_turning_radius: 0.0
#      lookup_table_size: 50
#      cost_scaling_factor: 5.0
#      reverse_penalty: 1.0
#      change_penalty: 0.0
#      non_straight_penalty: 1.05
#      cost_quizzie_window: 5
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.0
      use_astar: true
      allow_unknown: true


behavior_server:
  ros__parameters:
    use_sim_time: false
    global_frame: odom
    robot_base_frame: footprint_link
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
      sim_time: false
      min_rotational_vel: 0.05
      max_rotational_vel: 0.20
      rotational_acc_lim: 0.5
    backup:
      plugin: "nav2_behaviors/BackUp"
      sim_time: false
      backup_dist: 0.12
      backup_speed: 0.40
    wait:
      plugin: "nav2_behaviors/Wait"
      sim_time: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 15.0
      publish_frequency: 15.0
      global_frame: odom
      robot_base_frame: footprint_link
      transform_tolerance: 0.05 #0.15
      use_sim_time: false
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      footprint: "[ [0.144, 0.494], [0.144, -0.128], [-0.534, -0.128], [-0.534, 0.494] ]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          clearing: True
          marking: True
          observation_persistence: 0.5
          data_type: "LaserScan"
          min_obstacle_height: -0.1 # -3.0
          max_obstacle_height: 0.5 #3.0
          raytrace_max_range: 4.5
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          # inf_is_valid: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 6.0
        inflation_radius: 0.2 #0.13 # 0.4

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      transform_tolerance: 0.4
      robot_base_frame: footprint_link
      use_sim_time: false
      rolling_window: trues
      width: 14
      height: 14
      resolution: 0.05
      footprint: "[ [0.339, 0.31], [0.339, -0.31], [-0.339, -0.31], [-0.339, 0.31] ]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          clearing: True
          marking: True
          observation_persistence: 0.4
          data_type: "LaserScan"
          min_obstacle_height: -0.1
          max_obstacle_height: 0.5
          obstacle_max_range: 8.0
          raytrace_max_range: 8.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 6.0
        inflation_radius: 0.2 # 0.23 # 0.42

controller_server:
  ros__parameters:
    use_sim_time: false
    odom_topic: /nav2_odom
    controller_frequency: 50.0
    failure_tolerance: 1.0
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["grab_goal_checker", "shift_goal_checker", "area3_goal_checker", "pose_goal_checker","stopped_goal_checker"]
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.01
      movement_time_allowance: 100.0

    grab_goal_checker:
      plugin: "nav2_controller::PositionGoalChecker"
      xy_goal_tolerance: 0.05
      stateful: true

    shift_goal_checker:
      plugin: "nav2_controller::PositionGoalChecker"
      xy_goal_tolerance: 0.05
      stateful: true

    area3_goal_checker:
      plugin: "nav2_controller::PositionGoalChecker"
      xy_goal_tolerance: 0.05
      stateful: true

    pose_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 3.15
      stateful: true
    stopped_goal_checker:
      plugin: "nav2_controller::StoppedGoalChecker"
      xy_goal_tolerance: 0.15    # 你想要的 0.15 米范围
      yaw_goal_tolerance: 3.15    # 约 30 度，按需调整
      trans_stopped_velocity: 0.025 # 线速度低于此值视为“已停止”，默认 0.25 偏松
      rot_stopped_velocity: 0.025   # 角速度低于此值视为“已停止”，默认 0.25 偏松
      stateful: true

    controller_plugins: ["TEB_Short", "MPPI_Long"]

    TEB_Short:
      plugin: "teb_local_planner::TebLocalPlannerROS"
      teb_autosize: 1.0
      dt_ref: 0.12
      dt_hysteresis: 0.03
      min_samples: 3
      max_samples: 12
      global_plan_overwrite_orientation: false
      allow_init_with_backwards_motion: true
      max_global_plan_lookahead_dist: 0.5
      global_plan_viapoint_sep: -1.0
      global_plan_prune_distance: 0.2
      exact_arc_length: false
      feasibility_check_no_poses: 4
      publish_feedback: false

      max_vel_x: 1.0
      max_vel_x_backwards: 1.0
      max_vel_y: 1.0
      max_vel_theta: 0.10
      acc_lim_x: 0.45
      acc_lim_y: 0.45
      acc_lim_theta: 0.20

      footprint_model:
        type: "polygon"
        vertices: "[[0.144, 0.494], [0.144, -0.128], [-0.534, -0.128], [-0.534, 0.494]]"

      xy_goal_tolerance: 0.20
      yaw_goal_tolerance: 0.524
      free_goal_vel: false

      min_obstacle_dist: 0.10
      inflation_dist: 0.16
      include_costmap_obstacles: true
      costmap_obstacles_behind_robot_dist: 0.6
      obstacle_poses_affected: 10
      include_dynamic_obstacles: false

      no_inner_iterations: 4
      no_outer_iterations: 3
      optimization_activate: true
      optimization_verbose: false
      penalty_epsilon: 0.05
      obstacle_cost_exponent: 1.0
      weight_max_vel_x: 2.5
      weight_max_vel_y: 2.5
      weight_max_vel_theta: 2.5
      weight_acc_lim_x: 1.0
      weight_acc_lim_y: 1.0
      weight_acc_lim_theta: 1.2
      weight_kinematics_nh: 0.0
      weight_kinematics_forward_drive: 0.0
      weight_kinematics_turning_radius: 0.0
      weight_optimaltime: 3.0
      weight_shortest_path: 0.0
      weight_obstacle: 30.0
      weight_inflation: 0.1
      weight_dynamic_obstacle: 0.0
      weight_dynamic_obstacle_inflation: 0.0
      weight_viapoint: 1.0
      weight_adapt_factor: 2.0

      enable_homotopy_class_planning: false
      enable_multithreading: false
      max_number_classes: 1
      selection_cost_hysteresis: 1.0
      selection_prefer_initial_plan: 0.95
      selection_obst_cost_scale: 1.0
      selection_alternative_time_cost: false

      roadmap_graph_samples: 5
      roadmap_graph_area_width: 1.0
      roadmap_graph_area_length_scale: 1.0
      h_signature_prescaler: 0.5
      h_signature_threshold: 0.1
      obstacle_heading_threshold: 0.45
      switching_blocking_period: 0.0
      viapoints_all_candidates: false
      delete_detours_backwards: true
      max_ratio_detours_duration_best_duration: 2.0
      visualize_hc_graph: false
      visualize_with_time_as_z_axis_scale: 0.0

      shrink_horizon_backup: false
      shrink_horizon_min_duration: 2.0
      oscillation_recovery: false
      oscillation_v_eps: 0.02
      oscillation_omega_eps: 0.02
      oscillation_recovery_min_duration: 2.0
      oscillation_filter_duration: 2.0

    MPPI_Long:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 20
      model_dt: 0.02
      batch_size: 1200
      vx_std: 0.4
      vy_std: 0.4
      wz_std: 0.25
      vx_max: 2.0
      vx_min: -2.0
      vy_max: 2.0
      vy_min: -2.0
      ax_max: 2.5    # 最大加速度 (m/s²)
      ax_min: -2.5   # 最大减速度 (m/s²)
      wz_max: 0.45
      iteration_count: 1
      prune_distance: 1.2
      transform_tolerance: 0.2
      temperature: 0.25
      gamma: 0.015
      motion_model: "Omni"
      visualize: false

      critics: ["ConstraintCritic", "ObstaclesCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "TwirlingCritic"]

      ConstraintCritic:
        plugin: "mppi::critics::ConstraintCritic"
        enabled: true
        cost_power: 1
        cost_weight: 4.0

      ObstaclesCritic:
        plugin: "mppi::critics::ObstaclesCritic"
        enabled: true
        cost_power: 1
        repulsion_weight: 3.0
        critical_weight: 25.0
        consider_footprint: true

      GoalCritic:
        plugin: "mppi::critics::GoalCritic"
        enabled: true
        cost_power: 2
        cost_weight: 3.0
        threshold_to_consider: 1.2

      GoalAngleCritic:
        plugin: "mppi::critics::GoalAngleCritic"
        enabled: false
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.08

      PathAlignCritic:
        plugin: "mppi::critics::PathAlignCritic"
        enabled: true
        cost_power: 1
        cost_weight: 4.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 2
        threshold_to_consider: 0.4
        offset_from_furthest: 5
        use_path_orientations: false

      PathFollowCritic:
        plugin: "mppi::critics::PathFollowCritic"
        enabled: true
        cost_power: 1
        cost_weight: 8.0
        offset_from_furthest: 3
        threshold_to_consider: 1.2

      TwirlingCritic:
        plugin: "mppi::critics::TwirlingCritic"
        enabled: true
        cost_power: 1
        cost_weight: 8.0
velocity_smoother:
  ros__parameters:
    plugin: "nav2_velocity_smoother/VelocitySmoother"
    use_sim_time: false
    odom_topic: "/nav2_odom"
    odom_duration: 0.1
    smoothing_frequency: 100.0
    max_velocity: [2.0, 2.0, 0.0]
    min_velocity: [-2.0, -2.0, -0.0]
    max_accel: [3.5, 3.5, 0.0]
    max_decel: [-4.0, -4.0, 0.0]
    feedback: "OPEN_LOOP"
    holonomic: true
    ode_solver: "RungeKutta2"
    smooth_time: 0.08
    frame_id: "footprint_link"
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

param_handler:
  ros__parameters:
    use_sim_time: false

nav2_controller:
  ros__parameters:
    use_sim_time: false

nav2_costmap_2d:
  ros__parameters:
    use_sim_time: false

nav2_planner:
  ros__parameters:
    use_sim_time: false

# ===== 以下为 mapping_nav.yaml 中 MPPI 文件里没有的节点，补充进来 =====

map_server:
  ros__parameters:
    use_sim_time: false
    frame_id: map
    topic_name: map

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: [simple_smoother]
    simple_smoother:
      plugin: nav2_smoother::SimpleSmoother
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: true
    waypoint_task_executor_plugin: wait_at_waypoint
    wait_at_waypoint:
      plugin: nav2_waypoint_follower::WaitAtWaypoint
      enabled: true
      waypoint_pause_duration: 0
"""

MAPPING_NAV_PARAMS_YAML = r"""
map_server:
  ros__parameters:
    use_sim_time: false
    frame_id: map
    topic_name: map

amcl:
  ros__parameters:
    use_sim_time: false
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: footprint_link
    scan_topic: /scan
    map_topic: /map
    tf_broadcast: true
    robot_model_type: nav2_amcl::OmniMotionModel
    alpha1: 0.35
    alpha2: 0.35
    alpha3: 0.35
    alpha4: 0.35
    alpha5: 0.35
    laser_model_type: likelihood_field
    laser_min_range: 0.45
    laser_max_range: 10.0
    laser_likelihood_max_dist: 2.0
    max_beams: 60
    min_particles: 500
    max_particles: 3000
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    transform_tolerance: 1.0
    update_min_a: 0.05
    update_min_d: 0.05
    z_hit: 0.5
    z_rand: 0.5
    sigma_hit: 0.2

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map

behavior_server:
  ros__parameters:
    use_sim_time: false
    global_frame: odom
    robot_base_frame: footprint_link
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    behavior_plugins: [spin, backup, wait]

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: map
      rolling_window: false
      resolution: 0.02
      track_unknown_space: true
      plugins: [static_layer, obstacle_layer, inflation_layer]
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        enabled: true
        map_topic: /map
        map_subscribe_transient_local: true
        subscribe_to_updates: false
      obstacle_layer:
        scan:
          topic: /scan
          data_type: LaserScan
          min_obstacle_height: -0.15
          max_obstacle_height: 4.0
          inf_is_valid: true

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: odom
      rolling_window: true
      resolution: 0.02
      obstacle_layer:
        scan:
          topic: /scan
          data_type: LaserScan
          min_obstacle_height: -0.15
          max_obstacle_height: 4.0
          inf_is_valid: true

controller_server:
  ros__parameters:
    use_sim_time: false
    grab_goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 3.15
      stateful: true
    MPPI_Long:
      vx_max: 1.2 #2.0
      vx_min: -1.2 #-2.0
      vy_max: 1.2 #2.0
      vy_min: -1.2 #-2.0
      wz_max: 0.25

    TEB_Short:
      max_vel_x: 0.2
      max_vel_x_backwards: 0.2
      max_vel_y: 0.2
      max_vel_theta: 0.25

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: [simple_smoother]
    simple_smoother:
      plugin: nav2_smoother::SimpleSmoother
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: true
    waypoint_task_executor_plugin: wait_at_waypoint
    wait_at_waypoint:
      plugin: nav2_waypoint_follower::WaitAtWaypoint
      enabled: true
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    feedback: OPEN_LOOP
    max_velocity: [1.2, 1.2, 0.0]
    min_velocity: [-1.2, -1.2, 0.0]
    max_accel: [3.5, 3.5, 0.0]
    max_decel: [-4.0, -4.0, -0.0]
    velocity_timeout: 1.0
"""


def _merge_parameters(target, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge_parameters(target[key], value)
        else:
            target[key] = value
    return target


def _remove_parameter_file(path):
    if os.path.exists(path):
        os.remove(path)


def generate_launch_description():
    package_share = get_package_share_directory('rc_bringup')
    config_dir = os.path.join(package_share, 'config', 'reality')
    map_yaml_file = os.path.join(package_share, 'map', 'map2.yaml')
    bt_xml_file = os.path.join(
        config_dir, 'navigate_to_pose_teb_mppi_selector.xml'
    )
    bt_through_poses_xml_file = os.path.join(config_dir, 'nav2_mult.xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    parameters = _merge_parameters(
        yaml.safe_load(NAV2_PARAMS_YAML),
        yaml.safe_load(MAPPING_NAV_PARAMS_YAML),
    )
    parameter_file = tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        prefix='rc_bringup_nav2_',
        suffix='.yaml',
        delete=False,
    )
    parameter_path = parameter_file.name
    with parameter_file:
        yaml.safe_dump(parameters, parameter_file, sort_keys=False)
    atexit.register(_remove_parameter_file, parameter_path)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=parameter_path,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': bt_xml_file,
                'default_nav_through_poses_bt_xml': bt_through_poses_xml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': launch.substitutions.Command([
                    'xacro ',
                    os.path.join(package_share, 'urdf', 'reality', 'model.xacro'),
                ]),
            }],
        ),
        Node(
            package='stm32_driver',
            executable='odom_cmd_vel_fuser',
            name='odom_cmd_vel_fuser',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_in_topic': '/Odometry',
                'cmd_vel_topic': '/cmd_vel',
                'odom_out_topic': '/nav2_odom',
            }],
        ),
        Node(
            package='stm32_driver',
            executable='stm32_driver_node',
            name='stm32_driver',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            output='screen',
            parameters=[{
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 20.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': os.path.join(config_dir, 'MID360_driver.json'),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'fast_lio.yaml'),
                {'use_sim_time': use_sim_time, 'pcd_save.pcd_save_en': False},
            ],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            output='screen',
            remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'footprint_link',
                'transform_tolerance': 0.1,
                'min_height': -0.55,
                'max_height': 4.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0023,
                'scan_time': 0.05,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ]

    navigation_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    for package, executable in navigation_nodes:
        remappings = []
        if executable in ('controller_server', 'behavior_server'):
            remappings = [('cmd_vel', '/cmd_vel_nav')]
        elif executable == 'velocity_smoother':
            remappings = [
                ('cmd_vel', '/cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel'),
            ]
        actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
        ))

    keyboard_actions = [
        Node(
            package='run_keyboard_test',
            executable='nav_to_pose',
            output='screen',
        ),
        Node(
            package='run_keyboard_test',
            executable='run_slope',
            output='screen',
        ),
    ]
    action_group = GroupAction([
        TimerAction(period=3.0, actions=[keyboard_actions[0]]),
        TimerAction(period=3.0, actions=[keyboard_actions[1]]),
    ])

    actions.extend([
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [executable for _, executable in navigation_nodes],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(package_share, 'rviz', 'default.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])

    return LaunchDescription(actions + [action_group])
