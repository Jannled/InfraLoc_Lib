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

	static rclc_parameter_server_t param_server;

	rcl_publisher_t positionPublisher;

	int createParameterServer();
	int createPositionMessage();

	#ifdef DEBUG_INFRA_BUCKETS
	rcl_publisher_t strengthPublisher;
	rcl_publisher_t strengthPublisher2;
	rcl_publisher_t strengthPublisher3;

	int createStrengthMessage();
	int createStrengthMessage2();
	int createStrengthMessage3();
	#endif

public:
	InfraNode();
	~InfraNode();

	const unsigned int spinMillis = 100;

	// Beacon positions
	static vec2 pos_a;
	static vec2 pos_b;
	static vec2 pos_c;

	static number_t ang_a_bc;
	static number_t ang_b_ac;
	static number_t ang_c_ab;

	static bool positionUpdated;

	int init();
	int update();

	#ifdef DEBUG_INFRA_BUCKETS
	int publishBucketStrength(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle);
	int publishBucketStrength2(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle);
	int publishBucketStrength3(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle);
	#endif

	int publishPositionMessage(const pos2 &pose);

	static void error_loop();

	/**
	 * Rebuild the cache, after the positions of the sender/beacons have changed
	*/
	static void updatePositions();

	pos2 calculatePosition(const number_t alpha, const number_t beta, const number_t gamma);
};

#endif // INFRALOC_MICRO_ROS_NODE_H