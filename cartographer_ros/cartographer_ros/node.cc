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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

}  // namespace

Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer),
      update_published_transform(true),
      last_announced_submap_index_(-1) {
  absl::MutexLock lock(&mutex_);
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  submap_announcement_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::StampedSubmapEntry>(
          kSubmapAnnouncementTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteTsdfMeshServiceName, &Node::HandleWriteTsdfMesh, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kEnableMapUpdateServiceName, &Node::HandleEnableMapUpdateState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kUseScanMatchingServiceName, &Node::HandleUseScanMatchingState, this));
  tsdf_mesh_marker_publisher_ = node_handle_.advertise<::visualization_msgs::Marker>(
      kTSDFMeshMarkerTopic, kLatestOnlyPublisherQueueSize);
  tsdf_points_marker_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      kTSDFPointsMarkerTopic, kLatestOnlyPublisherQueueSize);
  tsdf_slice_marker_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
      kTSDFSliceMarkerTopic, kLatestOnlyPublisherQueueSize);
  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kSubmapAnnouncementPeriodSec),
      &Node::PublishSubmapAnnouncement, this));
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),
        &Node::PublishLocalTrajectoryData, this);
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTSDFPublishPeriodSec), &Node::PublishTSDFMeshMarker, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTSDFPublishPeriodSec), &Node::PublishTSDFPointsMarker, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTSDFPublishPeriodSec), &Node::PublishTSDFSliceMarker, this));
}

Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return true;
}

bool Node::HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = TrajectoryStateToStatus(
      request.trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response.status.message;
    return true;
  }
  map_builder_bridge_.HandleTrajectoryQuery(request, response);
  return true;
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock lock(&mutex_);
  const cartographer_ros_msgs::SubmapList& submap_list =
      map_builder_bridge_.GetSubmapList();
  submap_list_publisher_.publish(submap_list);

  std::vector<geometry_msgs::TransformStamped> stamped_transforms;
  for (const auto& submap : submap_list.submap) {
    geometry_msgs::TransformStamped transform;
    transform.header = submap_list.header;
    transform.child_frame_id = "submap_" + std::to_string(submap.submap_index);
    transform.transform = ToGeometryMsgTransform(ToRigid3d(submap.pose));
    stamped_transforms.push_back(transform);
  }
  tf_broadcaster_.sendTransform(stamped_transforms);
}
void Node::PublishSubmapAnnouncement(
    const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock lock(&mutex_);
  const cartographer_ros_msgs::SubmapList& submap_list =
      map_builder_bridge_.GetSubmapList();
  if (submap_list.submap.empty()) return;
  if (int(submap_list.submap.back().submap_index) !=
      last_announced_submap_index_) {
    cartographer_ros_msgs::StampedSubmapEntry stamped_submap_entry;
    stamped_submap_entry.submap = submap_list.submap.back();
    stamped_submap_entry.header = submap_list.header;
    submap_announcement_publisher_.publish(stamped_submap_entry);
    last_announced_submap_index_ =
        int(stamped_submap_entry.submap.submap_index);
  }
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event) {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    bool publish_pointcloud =
        trajectory_data.local_slam_data->time != extrapolator.GetLastPoseTime();
    if (publish_pointcloud) {
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::TransformStamped stamped_transform_t_latest;
    geometry_msgs::TransformStamped stamped_transform_t_optimized;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time t_now = FromRos(ros::Time::now());
    const ::cartographer::common::Time t_optimized =
        trajectory_data.local_slam_data->time;

    stamped_transform_t_latest.header.stamp = ToRos(t_now);
    stamped_transform_t_optimized.header.stamp = ToRos(t_optimized);
    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(t_now)
            : trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();

    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local;

    if (trajectory_data.published_to_tracking != nullptr) {
      if (update_published_transform ||
          last_updated_published_transforms_.find(entry.first) ==
              last_updated_published_transforms_.end()) {
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          std::vector<geometry_msgs::TransformStamped> stamped_transforms;

          stamped_transform_t_latest.header.frame_id = node_options_.map_frame;
          stamped_transform_t_latest.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform_t_latest.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map);
          stamped_transforms.push_back(stamped_transform_t_latest);

          stamped_transform_t_latest.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform_t_latest.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform_t_latest.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform_t_latest);

          tf_broadcaster_.sendTransform(stamped_transforms);
        } else {
          std::unique_ptr<::cartographer::transform::Rigid3d>
              tf_transform_t_optimized =
                  map_builder_bridge_.sensor_bridge(entry.first)
                      ->tf_bridge()
                      .LookupToTracking(
                          t_optimized,
                          entry.second.trajectory_options.published_frame);
          if (last_latest_tf_publish_time < t_now) {
            stamped_transform_t_latest.header.frame_id =
                node_options_.map_frame;
            stamped_transform_t_latest.child_frame_id =
                trajectory_data.trajectory_options.published_frame;
            stamped_transform_t_latest.transform = ToGeometryMsgTransform(
                tracking_to_map * (*tf_transform_t_optimized));
            tf_broadcaster_.sendTransform(stamped_transform_t_latest);
            last_latest_tf_publish_time = t_now;
          }
          if (last_optimized_tf_publish_time < t_optimized) {
            stamped_transform_t_optimized.header.frame_id =
                trajectory_data.trajectory_options.published_frame;
            stamped_transform_t_optimized.child_frame_id =
                node_options_.map_frame + "_optimized";
            stamped_transform_t_optimized.transform = ToGeometryMsgTransform(
                (tracking_to_map * (*tf_transform_t_optimized)).inverse());
            tf_broadcaster_.sendTransform(stamped_transform_t_optimized);
            last_optimized_tf_publish_time = t_optimized;
          }
        }
        last_updated_published_transforms_[entry.first] =
            stamped_transform_t_latest;
      } else {
        geometry_msgs::TransformStamped stamped_transform =
            last_updated_published_transforms_[entry.first];
        stamped_transform.header.stamp =
            node_options_.use_pose_extrapolator
                ? ToRos(t_now)
                : ToRos(trajectory_data.local_slam_data->time);
        tf_broadcaster_.sendTransform(stamped_transform);
      }
    }
    if (publish_pointcloud) {
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());
        for (const cartographer::sensor::RangefinderPoint point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
              point, 0.f /* time */));
        }

        sensor_msgs::PointCloud2 cloud_in_world = ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame + "_optimized",
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>()));
        sensor_msgs::PointCloud2 cloud_in_sensor_frame;
        try {
          geometry_msgs::TransformStamped transform =
              map_builder_bridge_.tf_buffer_->lookupTransform(
                  trajectory_data.trajectory_options.matched_pointcloud_frame,
                  node_options_.map_frame, cloud_in_world.header.stamp,
                  ros::Duration(1.0));
          tf2::doTransform(cloud_in_world, cloud_in_sensor_frame, transform);
          scan_matched_point_cloud_publisher_.publish(cloud_in_sensor_frame);
        } catch (const tf2::TransformException& ex) {
          ROS_ERROR("%s", ex.what());
        }
      }
    }
  }
}

void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}

void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

void Node::PublishTSDFMeshMarker(const ::ros::WallTimerEvent& unused_timer_event) {
  if (tsdf_mesh_marker_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    auto msg = map_builder_bridge_.GetTSDFMeshMarker();
    tsdf_mesh_marker_publisher_.publish(msg);
  }
}

void Node::PublishTSDFPointsMarker(const ::ros::WallTimerEvent& unused_timer_event) {
  if (tsdf_points_marker_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    auto msg = map_builder_bridge_.GetTSDFPointsMarker();
    tsdf_points_marker_publisher_.publish(msg);
  }
}

void Node::PublishTSDFSliceMarker(const ::ros::WallTimerEvent& unused_timer_event) {
  if (tsdf_slice_marker_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    auto msg = map_builder_bridge_.GetTSDFSliceMarker();
    tsdf_slice_marker_publisher_.publish(msg);
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, trajectory_id);
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTopicMismatchCheckDelaySec),
      &Node::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                &node_handle_, this),
         kImuTopic});
  }

  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  &node_handle_, this),
         kOdometryTopic});
  }
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             &node_handle_, this),
         kNavSatFixTopic});
  }
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             &node_handle_, this),
         kLandmarkTopic});
  }

  update_published_transform_subscriber_ = node_handle()->subscribe(
      "update_published_transform", 1000,
      &Node::HandleUpdatePublishedTransformMessage, this);
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_.GetTrajectoryStates();
  cartographer_ros_msgs::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message =
        absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message =
      absl::StrCat("Trajectory ", trajectory_id, " is in '",
                   TrajectoryStateToString(it->second), "' state.");
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::StatusCode::OK
          : cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to finish.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.shutdown();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      absl::StrCat("Finished trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  return status_response;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = LoadOptions(
      request.configuration_directory, request.configuration_basename);

  if (request.use_initial_pose) {
    const auto pose = ToRigid3d(request.initial_pose);
    if (!pose.IsValid()) {
      response.status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response.status.message;
      response.status.code =
          cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    response.status = TrajectoryStateToStatus(
        request.relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response.status.message;
      return true;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request.relative_to_trajectory_id);
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(ros::Time(0))));
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response.status.message = "Invalid trajectory options.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response.status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else {
    response.status.message = "Success.";
    response.trajectory_id = AddTrajectory(trajectory_options);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options);
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::HandleGetTrajectoryStates(
    ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
    ::cartographer_ros_msgs::GetTrajectoryStates::Response& response) {
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response.status.code = ::cartographer_ros_msgs::StatusCode::OK;
  response.trajectory_states.header.stamp = ros::Time::now();
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    response.trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

bool Node::HandleEnableMapUpdateState(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response) {
  absl::MutexLock lock(&mutex_);
  LOG(INFO) << "Setting UpdateMap to " << bool(request.data);
  map_builder_bridge_.map_builder_->SetMapUpdateEnabled(bool(request.data));
  response.success = true;
  return true;
}
bool Node::HandleUseScanMatchingState(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response) {
  absl::MutexLock lock(&mutex_);
  LOG(INFO) << "Setting UseScanMatching to " << bool(request.data);
  map_builder_bridge_.map_builder_->UseScanMatching(bool(request.data));
  response.success = true;
  return true;
}

bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_.SerializeState(request.filename,
                                         request.include_unfinished_submaps)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message =
        absl::StrCat("State written to '", request.filename, "'.");
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message =
        absl::StrCat("Failed to write '", request.filename, "'.");
  }
  return true;
}

bool Node::HandleReadMetrics(
    ::cartographer_ros_msgs::ReadMetrics::Request& request,
    ::cartographer_ros_msgs::ReadMetrics::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.timestamp = ros::Time::now();
  if (!metrics_registry_) {
    response.status.code = cartographer_ros_msgs::StatusCode::UNAVAILABLE;
    response.status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(&response);
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
  response.status.message = "Successfully read metrics.";
  return true;
}

bool Node::HandleWriteTsdfMesh(
    ::cartographer_ros_msgs::WriteTsdfMesh::Request &request,
    ::cartographer_ros_msgs::WriteTsdfMesh::Response &response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_.WriteTSDFMesh(request.filename, request.min_weight)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message =
        absl::StrCat("State written to '", request.filename, "'.");
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message =
        absl::StrCat("Failed to write '", request.filename, "'.");
  }
  return true;
}

void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  if (last_odometer_data_time >= FromRos(msg->header.stamp)) {
    ROS_WARN("Dropping odom data to maintain time consistency.");
    return;
  } else {
    last_odometer_data_time = FromRos(msg->header.stamp);
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleUpdatePublishedTransformMessage(const std_msgs::Bool& msg) {
  update_published_transform = msg.data;
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  if (last_imu_data_time >= FromRos(msg->header.stamp)) {
    ROS_WARN("Dropping imu data to maintain time consistency.");
    return;
  } else {
    last_imu_data_time = FromRos(msg->header.stamp);
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  if (last_range_data_time >= FromRos(msg->header.stamp)) {
    ROS_WARN("Dropping pointcloud to maintain time consistency.");
    return;
  } else {
    last_range_data_time = FromRos(msg->header.stamp);
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  if (last_range_data_time >= FromRos(msg->header.stamp)) {
    ROS_WARN("Dropping pointcloud to maintain time consistency.");
    return;
  } else {
    last_range_data_time = FromRos(msg->header.stamp);
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  if (last_range_data_time >= FromRos(msg->header.stamp)) {
    ROS_WARN("Dropping pointcloud to maintain time consistency.");
    return;
  } else {
    last_range_data_time = FromRos(msg->header.stamp);
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_.SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}

void Node::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& unused_timer_event) {
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);
  std::set<std::string> published_topics;
  std::stringstream published_topics_string;
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}

}  // namespace cartographer_ros
