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

#include <cmath>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <iostream>

#include "ros/ros.h"
#include "cartographer/common/port.h"
#include "cartographer/ground_truth/autogenerate_ground_truth_no_lc.h"
#include "cartographer/ground_truth/proto/relations.pb.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/transform/transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph used to generate"
              " and publish ground truth trajectory.");

std::string pose_graph_filename_;

namespace cartographer {
namespace ground_truth {
namespace {

void save_serialized_trajectory(int sig) {
  mapping::proto::PoseGraph pose_graph =
      io::DeserializePoseGraphFromFile(pose_graph_filename_);

  std::string serialized_data;
  pose_graph.SerializeToString(&serialized_data);
  std::ofstream out(pose_graph_filename_ + "_poses");
  out << serialized_data;
  out.close();
  LOG(INFO) << "Finished writing gt trajectory to: "
            << pose_graph_filename_ + "_poses";
  ros::shutdown();
}

void RunPublisher() {
  namespace fs = boost::filesystem;
  auto pose_graph_filename = fs::path(pose_graph_filename_).filename();
  auto pose_graph_parent = fs::path(pose_graph_filename_).parent_path();
  auto gt_filename = "gt_" + pose_graph_filename.filename().string();
  std::string gt_pose_graph_filename =
      pose_graph_parent.append(gt_filename).string();

  if (!fs::exists(gt_pose_graph_filename)) {
    LOG(ERROR) << "GT Pose Graph does not yet exist: " << gt_pose_graph_filename;
    return;
  }

  LOG(INFO) << "Reading pose graph from '" << gt_pose_graph_filename << "' ...";
  mapping::proto::PoseGraph pose_graph;
  try {
     pose_graph = io::DeserializePoseGraphFromFile(
        gt_pose_graph_filename);
  } catch (...) {
    LOG(ERROR) << "Could not read pose graph for: " << gt_pose_graph_filename;
    return;
  }
  ros::NodeHandle n;

  ros::Publisher gt_pub = n.advertise<visualization_msgs::MarkerArray>(
      "/trajectory_node_list_gt", 10);

  auto gt_marker_array = visualization_msgs::MarkerArray();
  auto gt_marker = visualization_msgs::Marker();
  gt_marker.header = std_msgs::Header();
  gt_marker.header.frame_id = "world";
  gt_marker.pose.orientation.w = 1;
  gt_marker.scale.x = 0.07;
  gt_marker.pose.position.z = 0.05;
  gt_marker.color.a = 0.75;
  gt_marker.color.r = 1;
  gt_marker.color.g = 101;
  gt_marker.color.b = 80;
  gt_marker.id = 2;
  gt_marker.type = 4;
  gt_marker.ns = "Trajectory 0";
  const mapping::proto::Trajectory& trajectory = pose_graph.trajectory(0);
  for (auto node : trajectory.node()) {
    geometry_msgs::Point point;
    point.x = node.pose().translation().x();
    point.y = node.pose().translation().y();
    point.z = node.pose().translation().z();
    gt_marker.points.push_back(point);
  }
  gt_marker_array.markers.push_back(gt_marker);

  while(ros::ok()) {
    gt_pub.publish(gt_marker_array);
    ros::Duration(1).sleep();
  }

}

}  // namespace
}  // namespace ground_truth
}  // namespace cartographer

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "\n\n"
      "This program reads the trajectory of a pbstream and publishes it as a "
      "marker array with topic /trajectory_node_list_gt .\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pose_graph_filename.empty()) {
    google::ShowUsageWithFlagsRestrict(argv[0], "publish_gt_trajectory");
    return EXIT_FAILURE;
  }

  ::ros::init(argc, argv, "save_gt_trajectory");
  ros::NodeHandle nh;
  signal(SIGINT, cartographer::ground_truth::save_serialized_trajectory);
  ROS_INFO("Got filename: %s", FLAGS_pose_graph_filename.c_str());

  pose_graph_filename_ = FLAGS_pose_graph_filename + ".pbstream";
  ::cartographer::ground_truth::RunPublisher();
  LOG(INFO) << "Waiting for shutdown to write trajectory ...";
  ros::spin();
  return 0;
}
