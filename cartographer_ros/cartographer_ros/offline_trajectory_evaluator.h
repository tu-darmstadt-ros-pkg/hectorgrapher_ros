
#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OFFLINE_TRAJECTORY_EVALUATOR_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OFFLINE_TRAJECTORY_EVALUATOR_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer_ros/node_options.h"

namespace cartographer_ros {

using MapBuilderFactory =
    std::function<std::unique_ptr<::cartographer::mapping::MapBuilderInterface>(
        const ::cartographer::mapping::proto::MapBuilderOptions&)>;

void RunOfflineTrajectoryEvaluator(
    const MapBuilderFactory& map_builder_factory);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OFFLINE_TRAJECTORY_EVALUATOR_H
