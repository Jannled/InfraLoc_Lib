#ifndef INFRALOC_MICRO_ROS_NODE_H
#define INFRALOC_MICRO_ROS_NODE_H

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "mymath.hpp"
#include "InfraLoc.hpp"

class InfraNode
{
private:
	// Private variables
	rclc_executor_t executor;
	rclc_support_t support;
	rcl_allocator_t allocator;
	rcl_node_t node;
	rcl_publisher_t strengthPublisher;

	int createParameterServer();
	int createInfralocService();
	int createStrengthMessage();

public:
	InfraNode();
	~InfraNode();

	unsigned int spinMillis = 50;

	int init();
	int update();

	int publishBucketStrength(std::array<number_t, INFRALOC_NUM_CHANNELS> values);

	static void error_loop();
};


#endif // INFRALOC_MICRO_ROS_NODE_H