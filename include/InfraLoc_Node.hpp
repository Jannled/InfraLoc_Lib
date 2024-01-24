#ifndef INFRALOC_MICRO_ROS_NODE_H
#define INFRALOC_MICRO_ROS_NODE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int initInfraNode();
int updateInfraNode();

#endif // INFRALOC_MICRO_ROS_NODE_H