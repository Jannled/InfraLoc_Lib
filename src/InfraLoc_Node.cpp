#ifdef MICRO_ROS_ENABLED
#include "InfraLoc_Node.hpp"

#include <Arduino.h>

#include <micro_ros_platformio.h>

#include "infraloc_interfaces/msg/bucket_strength.h"
#include "infraloc_interfaces/msg/infra_data.h"
#include "infraloc_interfaces/srv/beacon_angle.h"

#include <tracetools/tracetools.h>
#include <rcl/error_handling.h>

#include "rcl/publisher.h"
#include <rmw/error_handling.h>

// https://github.com/ros2/common_interfaces/tree/rolling/geometry_msgs/msg
#include "geometry_msgs/msg/pose2_d.h"

#define NUM_HANDLES_NEEDED (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1)

bool on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context);

// Only works for input angles 0 - 2π (0° - 360°), to avoid having to implement the formula with divisions
#define SMALLER_ANGLE(x) ((x) > M_PI ? ((x) - M_TWOPI) : (x))

// Constructor. Not really used
InfraNode::InfraNode()
{
	
}

// Destructor. Should never be called
InfraNode::~InfraNode()
{
	// Clean up
	rclc_executor_fini(&this->executor);
	rclc_support_fini(&this->support);
	rclc_parameter_server_fini(&param_server, &node);

	error_loop();
}

// Error handle loop
void InfraNode::error_loop() {
	while(1) {
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
		digitalWrite(LED_BUILTIN, HIGH);
	}
}

int InfraNode::init()
{
	set_microros_serial_transports(Serial);

	allocator = rcl_get_default_allocator();

	//create init_options
	// The Program will enter error state at this point, 
	// if no Serial connection could be established
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "infraloc", "", &support));

	// Executor init example with the minimum RCLC executor handles required
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(
		&executor, &support.context,
		NUM_HANDLES_NEEDED, &allocator
	));

	createParameterServer();
	createStrengthMessage();
	createPositionMessage();

	createStrengthMessage2();
	createStrengthMessage3();

	//createRawReadingsMessage();

	return RCL_RET_OK;
}

int InfraNode::update()
{
	// Spin executor to receive requests
	return rclc_executor_spin_some(&executor, RCL_MS_TO_NS(spinMillis));
}

void callback_beacon_angle(const void *request_msg, void *response_msg)
{
	// Cast messages to expected types
	infraloc_interfaces__srv__BeaconAngle_Request *request =
		(infraloc_interfaces__srv__BeaconAngle_Request *)request_msg;
	infraloc_interfaces__srv__BeaconAngle_Response *response =
		(infraloc_interfaces__srv__BeaconAngle_Response *)response_msg;

	// Handle request message and set the response message values
	const uint32_t k = request->k;
	response->angle = k;
}

/**
 * @brief TODO
 * @return 
 */
int InfraNode::createParameterServer()
{
	const rclc_parameter_options_t options = {
		.notify_changed_over_dds = true,
		.max_params = 8,
		.allow_undeclared_parameters = false,
		.low_mem_mode = false 
	};

	// Initialize parameter server with default configuration
	rcl_ret_t rc = rclc_parameter_server_init_with_option(&param_server, &node, &options);

	if(rc != RCL_RET_OK)
		return rc;

	// https://stackoverflow.com/questions/400257/how-can-i-pass-a-class-member-function-as-a-callback
	rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);

	if(rc != RCL_RET_OK)
		return rc;

	// Add parameter to the server
  	rc = rclc_add_parameter(&param_server, "chan_1_x", RCLC_PARAMETER_DOUBLE);
	rc = rclc_add_parameter(&param_server, "chan_1_y", RCLC_PARAMETER_DOUBLE);
	rc = rclc_add_parameter(&param_server, "chan_2_x", RCLC_PARAMETER_DOUBLE);
	rc = rclc_add_parameter(&param_server, "chan_2_y", RCLC_PARAMETER_DOUBLE);
	rc = rclc_add_parameter(&param_server, "chan_3_x", RCLC_PARAMETER_DOUBLE);
	rc = rclc_add_parameter(&param_server, "chan_3_y", RCLC_PARAMETER_DOUBLE);

	return rc;
}

int InfraNode::createStrengthMessage()
{
	const char* topic_name = "bucket_strength";

	// Get message type support
	const rosidl_message_type_support_t* type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(infraloc_interfaces, msg, BucketStrength);

	// Creates a reliable rcl publisher
	rcl_ret_t rc = rclc_publisher_init_best_effort(
		&strengthPublisher, &node, type_support, topic_name
	);

	return rc;
}

int InfraNode::createStrengthMessage2()
{
	const char* topic_name = "bucket_strength2";

	// Get message type support
	const rosidl_message_type_support_t* type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(infraloc_interfaces, msg, BucketStrength);

	// Creates a reliable rcl publisher
	rcl_ret_t rc = rclc_publisher_init_best_effort(
		&strengthPublisher2, &node, type_support, topic_name
	);

	return rc;
}

int InfraNode::createStrengthMessage3()
{
	const char* topic_name = "bucket_strength3";

	// Get message type support
	const rosidl_message_type_support_t* type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(infraloc_interfaces, msg, BucketStrength);

	// Creates a reliable rcl publisher
	rcl_ret_t rc = rclc_publisher_init_best_effort(
		&strengthPublisher3, &node, type_support, topic_name
	);

	return rc;
}

int InfraNode::createPositionMessage()
{
	const char* topic_name = "pose";

	/* 
	 * In theory, Pose2D is deprecated. However I don't want to send a 
	 * z-component plus a quaternion from a resource constrained device if they
	 * are unnecessary...
	*/

	// Get message type support
	const rosidl_message_type_support_t* type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D);

	// Creates a reliable rcl publisher
	rcl_ret_t rc = rclc_publisher_init_best_effort(
		&positionPublisher, &node, type_support, topic_name
	);

	return rc;
}

int InfraNode::publishBucketStrength(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle)
{
	infraloc_interfaces__msg__BucketStrength msg;
	for(size_t i=0; i<INFRALOC_NUM_CHANNELS; i++)
		msg.bucket_strength[i] = values.at(i);
	msg.angle = angle;

	return rcl_publish(&strengthPublisher, &msg, NULL);
}

int InfraNode::publishBucketStrength2(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle)
{
	infraloc_interfaces__msg__BucketStrength msg;
	for(size_t i=0; i<INFRALOC_NUM_CHANNELS; i++)
		msg.bucket_strength[i] = values.at(i);
	msg.angle = angle;

	return rcl_publish(&strengthPublisher2, &msg, NULL);
}

int InfraNode::publishBucketStrength3(std::array<number_t, INFRALOC_NUM_CHANNELS> values, number_t angle)
{
	infraloc_interfaces__msg__BucketStrength msg;
	for(size_t i=0; i<INFRALOC_NUM_CHANNELS; i++)
		msg.bucket_strength[i] = values.at(i);
	msg.angle = angle;

	return rcl_publish(&strengthPublisher3, &msg, NULL);
}

int InfraNode::publishPositionMessage(const pos2 &pose)
{
	geometry_msgs__msg__Pose2D msg;

	msg.x = pose.x;
	msg.y = pose.y;
	msg.theta = pose.theta;

	return rcl_publish(&positionPublisher, &msg, NULL);
}

int InfraNode::createRawReadingsMessage()
{
	const char* topic_name = "infra_data";

	// Get message type support
	const rosidl_message_type_support_t* type_support =
		ROSIDL_GET_MSG_TYPE_SUPPORT(infraloc_interfaces, msg, InfraData);

	// Creates a reliable rcl publisher
	rcl_ret_t rc = rclc_publisher_init_best_effort(
		&infraDataPublisher, &node, type_support, topic_name
	);

	return rc;
}

int InfraNode::publishRawReadings(const number_t* values, const size_t numSamples)
{
	infraloc_interfaces__msg__InfraData msg;
	memcpy(&msg.data, values, numSamples * sizeof(number_t));

	return rcl_publish(&infraDataPublisher, &msg, NULL);
}

pos2 InfraNode::calculatePosition(const number_t angle_a, const number_t angle_b, const number_t angle_c)
{
	volatile number_t alpha = abs(SMALLER_ANGLE(angle_b - angle_c));
	volatile number_t beta = abs(SMALLER_ANGLE(angle_c - angle_a));
	volatile number_t gamma = abs(SMALLER_ANGLE(angle_a - angle_b));

	vec2 pos = tienstraMethod(pos_a, pos_b, pos_c, alpha, beta, gamma, ang_a_bc, ang_b_ac, ang_c_ab);
	return {pos.x, pos.y, -42};
}

void InfraNode::updatePositions()
{
	// Clear the position updated flag. Using a flag to prevent multiple calls per `on_parameter_changed`
	positionUpdated = false;

	double chan_1_x = 0; 
	double chan_1_y = 0;
	double chan_2_x = 0;
	double chan_2_y = 0;
	double chan_3_x = 0;
	double chan_3_y = 0;

	rclc_parameter_get_double(&param_server, "chan_1_x", &chan_1_x);
	rclc_parameter_get_double(&param_server, "chan_1_y", &chan_1_y);
	rclc_parameter_get_double(&param_server, "chan_2_x", &chan_2_x);
	rclc_parameter_get_double(&param_server, "chan_2_y", &chan_2_y);
	rclc_parameter_get_double(&param_server, "chan_3_x", &chan_3_x);
	rclc_parameter_get_double(&param_server, "chan_3_y", &chan_3_y);

	pos_a.x = (number_t) chan_1_x;
	pos_a.y = (number_t) chan_1_y;
	pos_b.x = (number_t) chan_2_x;
	pos_b.y = (number_t) chan_2_y;
	pos_c.x = (number_t) chan_3_x;
	pos_c.y = (number_t) chan_3_y;

	ang_a_bc = vec_angle({pos_a.x-pos_b.x, pos_a.y-pos_b.y}, {pos_a.x-pos_c.x, pos_a.y-pos_c.y});
	ang_b_ac = vec_angle({pos_b.x-pos_a.x, pos_b.y-pos_a.y}, {pos_b.x-pos_c.x, pos_b.y-pos_c.y});
	ang_c_ab = vec_angle({pos_c.x-pos_a.x, pos_c.y-pos_a.y}, {pos_c.x-pos_b.x, pos_c.y-pos_b.y});
}

bool on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context)
{
	if(old_param == NULL || new_param == NULL)
		return false;

	InfraNode::positionUpdated = true;
	return true;
}

// Init static members
rclc_parameter_server_t InfraNode::param_server;

vec2 InfraNode::pos_a;
vec2 InfraNode::pos_b;
vec2 InfraNode::pos_c;

number_t InfraNode::ang_a_bc;
number_t InfraNode::ang_b_ac;
number_t InfraNode::ang_c_ab;

bool InfraNode::positionUpdated;

#endif // ROS2_ENABLED
