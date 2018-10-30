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
  map_frame = "map",
  tracking_frame = "base_imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_nav_sat = true,
  use_odometry = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  fixed_frame_pose_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 12

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- set these let ceres scan matcher to not trust the priors
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5

-- enlarge size of scan used for constructing each submap
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 5e1

-- set resolution of each scan to be 10 cm per grid
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 2e-1

-- set the range of rangefinder to be used in scan matching
TRAJECTORY_BUILDER_3D.min_range = 5

-- range data will only be added to submap if greater than max_distance_meters
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 1./18.

-- turns off residual histogram
POSE_GRAPH.log_residual_histograms = false

-- other configurations for global slam
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e1
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 5e1

-- settings for gps to cause no z-axis misalignment
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 0

-- settings to find better pose graph solution
POSE_GRAPH.optimize_every_n_nodes = 5e1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 4e1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(5)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.
POSE_GRAPH.constraint_builder.min_score = 5e-1
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 25
POSE_GRAPH.constraint_builder.sampling_ratio = 5e-1

-- setting to cause closeby trajectory to find constraints along z-axis
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 3e4
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 1
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1e3
POSE_GRAPH.global_sampling_ratio = 3e-1


return options
