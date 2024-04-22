#ifndef INFRALOC_MICRO_ROS_NODE_H
#define INFRALOC_MICRO_ROS_NODE_H

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){InfraNode::error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

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

	rclc_parameter_server_t param_server;

	rcl_publisher_t strengthPublisher;

	int createParameterServer();
	int createStrengthMessage();

	// Duplicates Quick n Dirty
	rcl_publisher_t strengthPublisher2;
	rcl_publisher_t strengthPublisher3;
	rcl_publisher_t infraDataPublisher;

public:
	InfraNode();
	~InfraNode();

	const unsigned int spinMillis = 100;

	// Beacon positions
	double chan_1_x = 0;
	double chan_1_y = 0;
	double chan_2_x = 0;
	double chan_2_y = 0;
	double chan_3_x = 0;
	double chan_3_y = 0;

	int init();
	int update();

	int publishBucketStrength(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle);

	static void error_loop();

	// // Duplicates Quick n Dirty
	int createStrengthMessage2();
	int publishBucketStrength2(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle);

	int createStrengthMessage3();
	int publishBucketStrength3(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle);

	int createRawReadingsMessage();
	int publishRawReadings(const number_t* values, const size_t numSamples);

	pos2 calculatePosition(number_t alpha, number_t beta, number_t gamma);
};

#endif // INFRALOC_MICRO_ROS_NODE_H