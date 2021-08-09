//
// Created by bastian on 02.08.21.
//

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_DYNAMIC_PARAMETERS_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_DYNAMIC_PARAMETERS_H_

#include <dynamic_reconfigure/server.h>
#include <cartographer_ros/dynamic_parametersConfig.h>

namespace cartographer_ros {

// Declare parameters as extern
extern double kTsdfMeshCutOffDistance;
extern double kTsdfMeshCutOffHeight;
extern double kTsdfCutOffDistance;
extern double kTsdfCutOffHeight;

/**
 * Callback function for dynamic reconfigure parameter server
 * @param config config that has changed. Struct can be accessed to get parameter values
 * @param level the result of ORing together all of level values of the parameters that have changed
 */
void dynamic_reconfigure_callback(cartographer_ros::dynamic_parametersConfig &config, uint32_t level);

}

#endif //CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_DYNAMIC_PARAMETERS_H_
