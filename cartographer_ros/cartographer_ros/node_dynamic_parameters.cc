//
// Created by bastian on 02.08.21.
//

#include <cartographer_ros/node_dynamic_parameters.h>

namespace cartographer_ros {

// Initialization of global (extern) parameter variables
bool kTsdfVisualizationHighRes;
double kTsdfMeshCutOffDistance;
double kTsdfMeshCutOffHeight;
double kTsdfCutOffDistance;
double kTsdfCutOffHeight;
double kTsdfSliceCutOffDistance;
double kTsdfSliceCenterX;
bool kTsdfSliceCenterXOnRobotPosition;
double kTsdfSliceCenterY;
bool kTsdfSliceCenterYOnRobotPosition;
double kTsdfSliceCenterZ;
bool kTsdfSliceCenterZOnRobotPosition;

void dynamic_reconfigure_callback(cartographer_ros::dynamic_parametersConfig &config, uint32_t level) {
  kTsdfVisualizationHighRes = config.tsdf_visualization_high_res;
  kTsdfMeshCutOffDistance = config.tsdf_mesh_cut_off_distance;
  kTsdfMeshCutOffHeight = config.tsdf_mesh_cut_off_height;
  kTsdfCutOffDistance = config.tsdf_cut_off_distance;
  kTsdfCutOffHeight = config.tsdf_cut_off_height;
  kTsdfSliceCutOffDistance = config.tsdf_slice_cut_off_distance;
  kTsdfSliceCenterX = config.tsdf_slice_center_x;
  kTsdfSliceCenterXOnRobotPosition = config.tsdf_slice_center_x_on_robot_position;
  kTsdfSliceCenterY = config.tsdf_slice_center_y;
  kTsdfSliceCenterYOnRobotPosition = config.tsdf_slice_center_y_on_robot_position;
  kTsdfSliceCenterZ = config.tsdf_slice_center_z;
  kTsdfSliceCenterZOnRobotPosition = config.tsdf_slice_center_z_on_robot_position;

  ROS_INFO("Reconfigure Request: tsdf_visualization_high_res: %s, "
           "tsdf_mesh_cut_off_distance: %f, "
           "tsdf_mesh_cut_off_height: %f, "
           "tsdf_cut_off_distance: %f, "
           "tsdf_cut_off_height: %f, "
           "tsdf_slice_cut_off_height: %f, "
           "tsdf_slice_center_x: %f, "
           "tsdf_slice_center_x_on_robot_position: %s, "
           "tsdf_slice_center_y: %f, "
           "tsdf_slice_center_y_on_robot_position: %s, "
           "tsdf_slice_center_z: %f, "
           "tsdf_slice_center_z_on_robot_position: %s",
           config.tsdf_visualization_high_res ? "True" : "False",
           config.tsdf_mesh_cut_off_distance,
           config.tsdf_mesh_cut_off_height,
           config.tsdf_cut_off_distance,
           config.tsdf_cut_off_height,
           config.tsdf_slice_cut_off_distance,
           config.tsdf_slice_center_x,
           config.tsdf_slice_center_x_on_robot_position ? "True" : "False",
           config.tsdf_slice_center_y,
           config.tsdf_slice_center_y_on_robot_position ? "True" : "False",
           config.tsdf_slice_center_z,
           config.tsdf_slice_center_z_on_robot_position ? "True" : "False");
}

}
