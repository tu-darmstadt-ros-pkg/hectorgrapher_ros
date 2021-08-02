/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/map_builder_bridge.h"
#include <cartographer/common/time.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros/node_dynamic_parameters.h"

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.2;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = absl::StrCat("Trajectory ", trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index) {
  auto it = landmark_id_to_index->find(landmark_id);
  if (it == landmark_id_to_index->end()) {
    const int new_index = landmark_id_to_index->size();
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  return it->second;
}

visualization_msgs::Marker CreateLandmarkMarker(int landmark_index,
                                                const Rigid3d& landmark_pose,
                                                const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = "Landmarks";
  marker.id = landmark_index;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.scale.x = kLandmarkMarkerScale;
  marker.scale.y = kLandmarkMarkerScale;
  marker.scale.z = kLandmarkMarkerScale;
  marker.color = ToMessage(cartographer::io::GetColor(landmark_index));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace

MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer),
      node_options_(node_options) {}

void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  cartographer::io::ProtoStreamReader stream(state_filename);
  map_builder_->LoadState(&stream, load_frozen_state);
}

int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      [this](const int trajectory_id, const ::cartographer::common::Time time,
             const Rigid3d local_pose,
             ::cartographer::sensor::RangeData range_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      trajectory_options.handle_scan_as_structured_cloud,
      node_options_.lookup_transform_timeout_sec, tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id));
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();
}

bool MapBuilderBridge::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps) {
  return map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                            filename);
}

void MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }

  response.submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
}

std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
  // Add active trajectories that are not yet in the pose graph, but are e.g.
  // waiting for input sensor data and thus already have a sensor bridge.
  for (const auto& sensor_bridge : sensor_bridges_) {
    trajectory_states.insert(std::make_pair(
        sensor_bridge.first,
        ::cartographer::mapping::PoseGraph::TrajectoryState::ACTIVE));
  }
  return trajectory_states;
}

cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::SubmapEntry submap_entry;
    submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(
        submap_id_pose.id.trajectory_id);
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_entry.start_stamp = ToRos(submap_id_pose.data.start_time);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}

std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
  std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
    {
      absl::MutexLock lock(&mutex_);
      if (local_slam_data_.count(trajectory_id) == 0) {
        continue;
      }
      local_slam_data = local_slam_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    local_trajectory_data[trajectory_id] = {
        local_slam_data,
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return local_trajectory_data;
}

void MapBuilderBridge::HandleTrajectoryQuery(
    cartographer_ros_msgs::TrajectoryQuery::Request& request,
    cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  // This query is safe if the trajectory doesn't exist (returns 0 poses).
  // However, we can filter unwanted states at the higher level in the node.
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  for (const auto& node_id_data :
       node_poses.trajectory(request.trajectory_id)) {
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = node_options_.map_frame;
    pose_stamped.header.stamp =
        ToRos(node_id_data.data.constant_pose_data.value().time);
    pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
    response.trajectory.push_back(pose_stamped);
  }
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = absl::StrCat(
      "Retrieved ", response.trajectory.size(),
      " trajectory nodes from trajectory ", request.trajectory_id, ".");
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }
  const auto constraints = map_builder_->pose_graph()->constraints();
  for (const auto& constraint : constraints) {
    if (constraint.tag ==
        cartographer::mapping::PoseGraph::Constraint::INTER_SUBMAP) {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                                                             .trajectory_id] =
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
      }
    }
  }

  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame);
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      last_inter_submap_constrained_node =
          (--node_poses.trajectory(trajectory_id).end())->id.node_index;
      last_inter_trajectory_constrained_node =
          last_inter_submap_constrained_node;
    }

    marker.color.a = 1.0;
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);

      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }
  }
  return trajectory_node_list;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetLandmarkPosesList() {
  visualization_msgs::MarkerArray landmark_poses_list;
  const std::map<std::string, Rigid3d> landmark_poses =
      map_builder_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) {
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(
        GetLandmarkIndex(id_to_pose.first, &landmark_to_index_),
        id_to_pose.second, node_options_.map_frame));
  }
  return landmark_poses_list;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses =
      map_builder_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = map_builder_->pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraph::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      } else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}

pcl::PointXYZ MapBuilderBridge::InterpolateVertex(float isolevel,
                                                  pcl::PointXYZ p1,
                                                  pcl::PointXYZ p2,
                                                  float valp1,
                                                  float valp2) {
  float mu;
  pcl::PointXYZ p;

  // If jump is too big, return a point with x value NAN to indicate it's false
  if (std::abs(valp1 - valp2) > 0.95) {
    p.x = NAN;
    return p;
  }

  if (std::abs(isolevel - valp1) < 1e-5) {
    return p1;
  }
  if (std::abs(isolevel - valp2) < 1e-5) {
    return p2;
  }
  if (std::abs(valp1 - valp2) < 1e-5) {
    p.getArray3fMap() = 0.5 * (p1.getArray3fMap() + p2.getArray3fMap());
    return p;
  }
  mu = (isolevel - valp1) / (valp2 - valp1);
  p.x = p1.x + mu * (p2.x - p1.x);
  p.y = p1.y + mu * (p2.y - p1.y);
  p.z = p1.z + mu * (p2.z - p1.z);
  return p;
}

int MapBuilderBridge::ProcessCube(Cube &cube,
                                  pcl::PointCloud<pcl::PointXYZ> &cloud,
                                  float isolevel) {
  int cubeindex = 0;
  if (cube.tsd_values[0] <= isolevel) cubeindex |= 1;
  if (cube.tsd_values[1] <= isolevel) cubeindex |= 2;
  if (cube.tsd_values[2] <= isolevel) cubeindex |= 4;
  if (cube.tsd_values[3] <= isolevel) cubeindex |= 8;
  if (cube.tsd_values[4] <= isolevel) cubeindex |= 16;
  if (cube.tsd_values[5] <= isolevel) cubeindex |= 32;
  if (cube.tsd_values[6] <= isolevel) cubeindex |= 64;
  if (cube.tsd_values[7] <= isolevel) cubeindex |= 128;

  // Cube is entirely in/out of the surface
  if (edge_table_[cubeindex] == 0) {
    return 0;
  }
  pcl::PointXYZ vertices_list[12];

  // Find the points where the surface intersects the cube
  if (edge_table_[cubeindex] & 1) {
    vertices_list[0] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[0],
                          cube.vertice_pos_global[1],
                          cube.tsd_values[0],
                          cube.tsd_values[1]);
  }
  if (edge_table_[cubeindex] & 2) {
    vertices_list[1] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[1],
                          cube.vertice_pos_global[2],
                          cube.tsd_values[1],
                          cube.tsd_values[2]);
  }
  if (edge_table_[cubeindex] & 4) {
    vertices_list[2] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[2],
                          cube.vertice_pos_global[3],
                          cube.tsd_values[2],
                          cube.tsd_values[3]);
  }
  if (edge_table_[cubeindex] & 8) {
    vertices_list[3] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[3],
                          cube.vertice_pos_global[0],
                          cube.tsd_values[3],
                          cube.tsd_values[0]);
  }
  if (edge_table_[cubeindex] & 16) {
    vertices_list[4] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[4],
                          cube.vertice_pos_global[5],
                          cube.tsd_values[4],
                          cube.tsd_values[5]);
  }
  if (edge_table_[cubeindex] & 32) {
    vertices_list[5] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[5],
                          cube.vertice_pos_global[6],
                          cube.tsd_values[5],
                          cube.tsd_values[6]);
  }
  if (edge_table_[cubeindex] & 64) {
    vertices_list[6] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[6],
                          cube.vertice_pos_global[7],
                          cube.tsd_values[6],
                          cube.tsd_values[7]);
  }
  if (edge_table_[cubeindex] & 128) {
    vertices_list[7] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[7],
                          cube.vertice_pos_global[4],
                          cube.tsd_values[7],
                          cube.tsd_values[4]);
  }
  if (edge_table_[cubeindex] & 256) {
    vertices_list[8] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[0],
                          cube.vertice_pos_global[4],
                          cube.tsd_values[0],
                          cube.tsd_values[4]);
  }
  if (edge_table_[cubeindex] & 512) {
    vertices_list[9] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[1],
                          cube.vertice_pos_global[5],
                          cube.tsd_values[1],
                          cube.tsd_values[5]);
  }
  if (edge_table_[cubeindex] & 1024) {
    vertices_list[10] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[2],
                          cube.vertice_pos_global[6],
                          cube.tsd_values[2],
                          cube.tsd_values[6]);
  }
  if (edge_table_[cubeindex] & 2048) {
    vertices_list[11] =
        InterpolateVertex(isolevel,
                          cube.vertice_pos_global[3],
                          cube.vertice_pos_global[7],
                          cube.tsd_values[3],
                          cube.tsd_values[7]);
  }

  // Create the triangle
  int triangle_count = 0;
  pcl::PointXYZ triangle[3];
  for (int i = 0; triangle_table_[cubeindex][i] != -1; i += 3) {
    triangle[0] = vertices_list[triangle_table_[cubeindex][i]];
    triangle[1] = vertices_list[triangle_table_[cubeindex][i + 1]];
    triangle[2] = vertices_list[triangle_table_[cubeindex][i + 2]];
    if (isnan(triangle[0].x) || isnan(triangle[1].x) || isnan(triangle[2].x)) continue;
    cloud.push_back(triangle[0]);
    cloud.push_back(triangle[1]);
    cloud.push_back(triangle[2]);
    triangle_count++;
  }
  return (triangle_count);

}

visualization_msgs::Marker MapBuilderBridge::GetTSDFMesh() {
  ::cartographer::mapping::MapById<
      ::cartographer::mapping::SubmapId,
      ::cartographer::mapping::PoseGraphInterface::SubmapData>
      data = map_builder_->pose_graph()->GetAllSubmapData();
  visualization_msgs::Marker marker;

  if (!data.empty()) {
    const auto submap3d =
        static_cast<const ::cartographer::mapping::Submap3D *>(
            data.begin()->data.submap.get());
    const auto tsdf =
        static_cast<const ::cartographer::mapping::HybridGridTSDF *>(
            &submap3d->high_resolution_hybrid_grid());
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto local_trajectory_data = GetLocalTrajectoryData();
    auto robot_position = local_trajectory_data[0].local_slam_data->local_pose.translation();

    float resolution = tsdf->resolution();
    float isolevel = 0.0f;
    int count = 0;

    for (auto it = ::cartographer::mapping::HybridGridTSDF::Iterator(*tsdf);
         !it.Done(); it.Next()) {
      const ::cartographer::mapping::TSDFVoxel voxel = it.GetValue();
      const float tsd = tsdf->ValueConverter().ValueToTSD(voxel.discrete_tsd);
      const Eigen::Vector3f cell_center_submap = tsdf->GetCenterOfCell(it.GetCellIndex());
      const Eigen::Vector3f
          cell_center_global = submap3d->local_pose().cast<float>() * cell_center_submap;

      if (voxel.discrete_weight == 0) {
        // Skip inner-object voxels
        continue;
      }

      if ((robot_position.cast<float>() - cell_center_global).norm()
          > static_cast<float>(cartographer_ros::kTsdfMeshCutOffDistance)) {
        // Cut-off cells that are too far away from the robot
        continue;
      }

      if (cell_center_global.z() - static_cast<float>(robot_position.z())
          > static_cast<float>(cartographer_ros::kTsdfMeshCutOffHeight)) {
        // Cut-off cells that are too high above the robot
        continue;
      }

      Cube cube;
      for (int i = 0; i < 8; ++i) {
        cube.vertice_ids[i] = it.GetCellIndex();
        cube.vertice_ids[i].x() += cube.position_arr[i][0];
        cube.vertice_ids[i].y() += cube.position_arr[i][1];
        cube.vertice_ids[i].z() += cube.position_arr[i][2];
        cube.vertice_pos_global[i].x =
            cell_center_global.x() + static_cast<float>(cube.position_arr[i][0]) * resolution;
        cube.vertice_pos_global[i].y =
            cell_center_global.y() + static_cast<float>(cube.position_arr[i][1]) * resolution;
        cube.vertice_pos_global[i].z =
            cell_center_global.z() + static_cast<float>(cube.position_arr[i][2]) * resolution;
        cube.tsd_weights[i] = tsdf->GetWeight(cube.vertice_ids[i]);

        cube.tsd_values[i] =
            cube.tsd_weights[i] <= 0.0f ? tsd : tsdf->GetTSD(cube.vertice_ids[i]);
      }
      count += ProcessCube(cube, cloud, isolevel);

    }
    LOG(INFO) << "A total of " << count << " triangles are processed. Points in Cloud: "
              << cloud.size();

    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(cloud, mesh.cloud);

    for (uint32_t i = 0; i < count; i++) {
      pcl::Vertices v;
      v.vertices.push_back(i * 3 + 0);
      v.vertices.push_back(i * 3 + 1);
      v.vertices.push_back(i * 3 + 2);
      mesh.polygons.push_back(v);
    }

    std_msgs::ColorRGBA color;
    marker.header.frame_id = "world_cartographer";
    marker.header.stamp = ::ros::Time::now();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    for (auto &vertice_group : mesh.polygons) {
      for (auto &vertice : vertice_group.vertices) {
        geometry_msgs::Point temp_point;
        temp_point.x = cloud[vertice].x;
        temp_point.y = cloud[vertice].y;
        temp_point.z = cloud[vertice].z;
        marker.points.push_back(temp_point);
      }
      Eigen::Vector3f u = {cloud[vertice_group.vertices[1]].x - cloud[vertice_group.vertices[0]].x,
                           cloud[vertice_group.vertices[1]].y - cloud[vertice_group.vertices[0]].y,
                           cloud[vertice_group.vertices[1]].z - cloud[vertice_group.vertices[0]].z};
      Eigen::Vector3f v = {cloud[vertice_group.vertices[2]].x - cloud[vertice_group.vertices[0]].x,
                           cloud[vertice_group.vertices[2]].y - cloud[vertice_group.vertices[0]].y,
                           cloud[vertice_group.vertices[2]].z - cloud[vertice_group.vertices[0]].z};
      Eigen::Vector3f normal = u.cross(v).normalized();
      std_msgs::ColorRGBA surface_color;
      surface_color.r = (normal.x() + 1.0f) * 0.5f;
      surface_color.g = (normal.y() + 1.0f) * 0.5f;
      surface_color.b = (normal.z() + 1.0f) * 0.5f;
      surface_color.a = 1;
      marker.colors.push_back(surface_color);
      marker.colors.push_back(surface_color);
      marker.colors.push_back(surface_color);
    }
  }

  return marker;
}

sensor_msgs::PointCloud2 MapBuilderBridge::GetTSDF() {
  ::cartographer::mapping::MapById<
      ::cartographer::mapping::SubmapId,
      ::cartographer::mapping::PoseGraphInterface::SubmapData>
      data = map_builder_->pose_graph()->GetAllSubmapData();
  sensor_msgs::PointCloud2 msg;

  if (!data.empty()) {
    const ::cartographer::mapping::Submap3D *submap3d =
        static_cast<const ::cartographer::mapping::Submap3D *>(
            data.begin()->data.submap.get());
    const ::cartographer::mapping::HybridGridTSDF *tsdf =
        static_cast<const ::cartographer::mapping::HybridGridTSDF *>(
            &submap3d->high_resolution_hybrid_grid());
    std::vector<Eigen::Array4f> cells;
    auto local_trajectory_data = GetLocalTrajectoryData();
    auto robot_position = local_trajectory_data[0].local_slam_data->local_pose.translation();
    float resolution = tsdf->resolution();

    for (auto it = ::cartographer::mapping::HybridGridTSDF::Iterator(*tsdf); !it.Done();
         it.Next()) {
      const ::cartographer::mapping::TSDFVoxel voxel = it.GetValue();
      const float tsd = tsdf->ValueConverter().ValueToTSD(voxel.discrete_tsd);
      const Eigen::Vector3f cell_center_submap = tsdf->GetCenterOfCell(it.GetCellIndex());
      const Eigen::Vector3f
          cell_center_global = submap3d->local_pose().cast<float>() * cell_center_submap;

      if (voxel.discrete_weight == 0) {
        // Skip inner-object voxels
        continue;
      }

      if ((robot_position.cast<float>() - cell_center_global).norm()
          > static_cast<float>(cartographer_ros::kTsdfCutOffDistance)) {
        // Cut-off cells that are too far away from the robot
        continue;
      }

      if (cell_center_global.z() - static_cast<float>(robot_position.z())
          > static_cast<float>(cartographer_ros::kTsdfCutOffHeight)) {
        // Cut-off cells that are too high above the robot
        continue;
      }

      if (tsd >= 0 && tsd < resolution) {
        cells.emplace_back(cell_center_global[0], cell_center_global[1],
                           cell_center_global[2], tsd);
      }
    }

    msg = ToPointCloud2Message(
        ::cartographer::common::ToUniversal(FromRos(::ros::Time::now())),
        "world_cartographer", cells);
  }

  return msg;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local) {
  std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
      std::make_shared<LocalTrajectoryData::LocalSlamData>(
          LocalTrajectoryData::LocalSlamData{time, local_pose,
                                             std::move(range_data_in_local)});
  absl::MutexLock lock(&mutex_);
  local_slam_data_[trajectory_id] = std::move(local_slam_data);
}

}  // namespace cartographer_ros
