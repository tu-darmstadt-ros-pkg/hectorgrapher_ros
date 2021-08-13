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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_listener.h"
#include <dynamic_reconfigure/server.h>
#include <cartographer_ros/dynamic_parametersConfig.h>
#include "cartographer_ros/node_dynamic_parameters.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

class NodeWrapper {
 public:
  std::unique_ptr<tf2_ros::TransformListener> tf_;
  std::unique_ptr<Node> node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  ros::Subscriber syscommand_subscriber_;
  ros::NodeHandle node_handle_;

  NodeWrapper() {
    syscommand_subscriber_ = node_handle_.subscribe(
        "syscommand", 1000, &NodeWrapper::SyscommandCallback, this);
  }

  void Run() {
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf_buffer_ = absl::make_unique<tf2_ros::Buffer>(
        ::ros::Duration(kTfBufferCacheTimeInSeconds));
    tf_ = absl::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) = LoadOptions(
        FLAGS_configuration_directory, FLAGS_configuration_basename);

    auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
        node_options.map_builder_options);
    node_ = absl::make_unique<Node>(node_options, std::move(map_builder),
                                    tf_buffer_.get(), FLAGS_collect_metrics);
    if (!FLAGS_load_state_filename.empty()) {
      node_->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }

    if (FLAGS_start_trajectory_with_default_topics) {
      node_->StartTrajectoryWithDefaultTopics(trajectory_options);
    }
  }

  void Finish() {
    node_->FinishAllTrajectories();
    //    node_->RunFinalOptimization();
    //    if (!FLAGS_save_state_filename.empty()) {
    //      node_->SerializeState(FLAGS_save_state_filename,
    //                            true /* include_unfinished_submaps */);
    //    }
  }

  void SyscommandCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "reset_cartographer") {
      ROS_INFO("Resetting now due to syscommand.");
      Finish();
      node_.reset();
      tf_.reset();
      tf_buffer_.reset();
      Run();
      ROS_INFO("Finished reset.");
    }
  }
};

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  dynamic_reconfigure::Server<cartographer_ros::dynamic_parametersConfig> server;
  dynamic_reconfigure::Server<cartographer_ros::dynamic_parametersConfig>::CallbackType f;

  f = boost::bind(&cartographer_ros::dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::NodeWrapper node_wrapper;
  node_wrapper.Run();
  ::ros::spin();
  node_wrapper.Finish();
  ::ros::shutdown();
}
