-- Copyright 2016### The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

num_accumulated_range_data_ = 15

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "world",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,  -- false
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = num_accumulated_range_data_,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,  -- 1.
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = num_accumulated_range_data_  -- too big: too much delay
TRAJECTORY_BUILDER_2D.num_accumulated_range_data_points = 200  -- zero, if only the number of scans defined by "num_accumulated_range_data" should be checked
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- TRAJECTORY_BUILDER_2D.voxel_filter_size =  0.01
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length =  0.25
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points =  1000
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range =  100

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2 -- not too big: Slow and finds bad solutions sometimes! (Or penalty higher!)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(60.)  -- big safety factor for odom rotation accuracy
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.7
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1


-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.001;
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.001;

-- -- Disable submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10000000000
POSE_GRAPH.constraint_builder.sampling_ratio = 0.000000001
POSE_GRAPH.global_sampling_ratio = 0.000000001
POSE_GRAPH.optimize_every_n_nodes = 10000000000
POSE_GRAPH.constraint_builder.min_score = 1.93
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0

-- -- Enable submaps
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 15
-- POSE_GRAPH.optimize_every_n_nodes = 10
-- -- POSE_GRAPH.constraint_builder.min_score = 0.001
-- -- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1
-- -- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.1
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 0.1
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 0.01
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.)
-- POSE_GRAPH.optimization_problem.huber_scale = 10

-- Enable TSDF 
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "TSDF"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.use_pca = false
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.const_weight   = 0.1
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.tsdf_weight_scale   = 1  -- 0.0?
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sort_range_data  = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.num_normal_samples  = 10  -- 16 | 10
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sample_radius  = 0.3  -- 0.15

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance_update_factor  = 1.0
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space_only_first_hits = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal  = false  -- true  -> false: way more detail in rviz maps!
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_angle_scan_normal_to_ray_kernel_bandwith   = 0.0
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.min_normal_weight   = 0.5  -- 0.1
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance  = 0.1 -- 0.1 
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.maximum_weight  = 1024
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_distance_cell_to_hit_kernel_bandwith  = 10  -- 10
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.free_space_weight  = 0.1  -- 0.05

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight  = 0.1  -- 1? not so good
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight   = 0.001
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight_vertical   = 0.001
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight  = 0.0001
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.empty_space_cost  = 0.5  -- 0.5

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 500
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.use_gnc = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.max_iterations  = 80
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.non_convexity_stop  = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.gm_shape  = 2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.min_convexity  = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.non_convexity_inc_factor  = 1.4
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.gnc_options_2d.max_retries = 1

-- Old Settings:
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_2D"
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "TSDF"
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_angle_scan_normal_to_ray_kernel_bandwith = 0.
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_distance_cell_to_hit_kernel_bandwith = 50.
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space = false
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance = 0.3
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.use_pca = false
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sample_radius = 0.65
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.tsdf_weight_scale   = 0.0
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.const_weight   = 0.05
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.maximum_weight = 3500
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_distance_cell_to_hit_kernel_bandwith = 0.
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal  = true
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight   = 0.1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight  = 0.001
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight  = 0.1
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.free_space_weight  = 0.1
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.min_normal_weight  = 0.1
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sort_range_data   = false

return options
