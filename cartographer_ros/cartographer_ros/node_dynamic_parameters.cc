//
// Created by bastian on 02.08.21.
//

#include <cartographer_ros/node_dynamic_parameters.h>

namespace cartographer_ros {

// Initialization of global (extern) parameter variables
double kTsdfMeshCutOffDistance;
double kTsdfMeshCutOffHeight;
double kTsdfCutOffDistance;
double kTsdfCutOffHeight;

void dynamic_reconfigure_callback(cartographer_ros::dynamic_parametersConfig &config, uint32_t level) {
  kTsdfMeshCutOffDistance = config.tsdf_mesh_cut_off_distance;
  kTsdfMeshCutOffHeight = config.tsdf_mesh_cut_off_height;
  kTsdfCutOffDistance = config.tsdf_cut_off_distance;
  kTsdfCutOffHeight = config.tsdf_cut_off_height;

  ROS_INFO("Reconfigure Request: tsdf_mesh_cut_off_distance: %f, tsdf_mesh_cut_off_height: %f, "
           "tsdf_cut_off_distance: %f, tsdf_cut_off_height: %f",
           config.tsdf_mesh_cut_off_distance,
           config.tsdf_mesh_cut_off_height,
           config.tsdf_cut_off_distance,
           config.tsdf_cut_off_height);
}

}
