

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "world_cartographer",
  tracking_frame = "imu_link",
  published_frame = "odom",
  matched_pointcloud_frame = "spin_lidar_mount_link_fixed",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 5.,
  pose_publish_period_sec = 5e-2,
  trajectory_publish_period_sec = 1.,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  handle_scan_as_structured_cloud = true,
}


MAP_BUILDER.use_trajectory_builder_3d = true
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false

TRAJECTORY_BUILDER_3D.min_range  = 0.3
TRAJECTORY_BUILDER_3D.max_range = 60
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.1

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 10000000000
POSE_GRAPH.constraint_builder.sampling_ratio = 0.000000001
POSE_GRAPH.global_sampling_ratio = 0.000000001
POSE_GRAPH.optimize_every_n_nodes = 10000000000
POSE_GRAPH.constraint_builder.min_score = 1.93
POSE_GRAPH.log_residual_histograms = false



TRAJECTORY_BUILDER_3D.submaps.grid_type = "TSDF"
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_3D"
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.relative_truncation_distance  = 2.5
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal  = true
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.normal_computation_method  = "CLOUD_STRUCTURE"
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.insertion_ratio = 1.0
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.min_range = 0.5
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.max_range = 40.0
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.normal_computation_horizontal_stride = 5
TRAJECTORY_BUILDER_3D.submaps.high_resolution_range_data_inserter.tsdf_range_data_inserter.normal_computation_vertical_stride = 1


TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_3D"
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.relative_truncation_distance  = 2.5
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal  = true
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.normal_computation_method  = "CLOUD_STRUCTURE"
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.insertion_ratio = 1.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.min_range = 0.5
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.max_range = 40.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.normal_computation_horizontal_stride = 5
TRAJECTORY_BUILDER_3D.submaps.low_resolution_range_data_inserter.tsdf_range_data_inserter.normal_computation_vertical_stride = 1



TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.initialize_map_orientation_with_imu = true
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.calibrate_imu  = false
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.imu_integrator = "EULER"
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.imu_cost_term =  "PREINTEGRATION"
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.velocity_weight  = 0 --80
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.translation_weight  = 0 --100
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.rotation_weight = 0 --4
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.odometry_translation_weight  = 1.0 -- 1.0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.odometry_rotation_weight  = 20.0 -- 20.0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.low_resolution_grid_weight  = 2000000000000.0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.high_resolution_grid_weight  = 10.0 --250.0
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.use_adaptive_odometry_weights  = true
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.velocity_in_state = false



TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.initialization_duration  = 1.5
-- TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.optimization_rate =  0.1
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_horizon =  0.4
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_rate =  0.1 --0.1




TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.use_per_point_unwarping = true
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.use_multi_resolution_matching = true
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.num_points_per_subdivision = 10
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.sampling_max_delta_translation = 0.1
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.sampling_max_delta_rotation = 0.025
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.sampling_min_delta_time = 0.025
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.sampling_max_delta_time = 0.5
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.control_point_sampling =  "CONSTANT"



TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.initialization_duration  = 1.5
-- TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.optimization_rate =  0.05
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_horizon =  0.4
TRAJECTORY_BUILDER_3D.optimizing_local_trajectory_builder.ct_window_rate =  0.1 --0.1

return options


