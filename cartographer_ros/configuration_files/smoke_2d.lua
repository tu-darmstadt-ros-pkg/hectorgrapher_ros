-- Copyright 2016 The Cartographer Authors
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

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "world",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 1.0
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3



-- Disable submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10000000000
POSE_GRAPH.constraint_builder.sampling_ratio = 0.000000001
POSE_GRAPH.global_sampling_ratio = 0.000000001
POSE_GRAPH.optimize_every_n_nodes = 10000000000
POSE_GRAPH.constraint_builder.min_score = 1.0

-- -- Enable TSDF 
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "TSDF_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "TSDF"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.use_pca = false
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.const_weight   = 0.1
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.tsdf_weight_scale   = 0.0
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sort_range_data  = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.num_normal_samples  = 16
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.normal_estimation_options.sample_radius  = 0.15
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.project_sdf_distance_to_scan_normal  = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_angle_scan_normal_to_ray_kernel_bandwith   = 0.0
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.min_normal_weight   = 0.1
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.truncation_distance  = 0.07
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.maximum_weight  = 128
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.update_weight_distance_cell_to_hit_kernel_bandwith  = 10
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.tsdf_range_data_inserter.free_space_weight  = 0.5


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight  = 0.25
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight   = 0.1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight  = 0.001

return options
