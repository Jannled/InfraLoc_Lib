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

#define NUM_HANDLES_NEEDED (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1)

bool on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context);

InfraNode::InfraNode()
{
	
}

InfraNode::~InfraNode()
{
	// Clean up
	rclc_executor_fini(&this->executor);
	// other stuff
	rclc_support_fini(&this->support);

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
	//rc = rclc_add_parameter(&param_server, "chan_1_y", RCLC_PARAMETER_DOUBLE);
	//rc = rclc_add_parameter(&param_server, "chan_2_x", RCLC_PARAMETER_DOUBLE);
	//rc = rclc_add_parameter(&param_server, "chan_2_y", RCLC_PARAMETER_DOUBLE);
	//rc = rclc_add_parameter(&param_server, "chan_3_x", RCLC_PARAMETER_DOUBLE);
	//rc = rclc_add_parameter(&param_server, "chan_3_y", RCLC_PARAMETER_DOUBLE);

	rclc_parameter_set_double(&param_server, "chan_1_x", 1.0);

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

pos2 InfraNode::calculatePosition(number_t alpha, number_t beta, number_t gamma)
{
	const vec2 pos_a = {chan_1_x, chan_1_y};
	const vec2 pos_b = {chan_2_x, chan_2_y};
	const vec2 pos_c = {chan_3_x, chan_3_y};

	// Law of Cosines or Dot Product
	// Dot product is easier
	// https://muthu.co/using-the-law-of-cosines-and-vector-dot-product-formula-to-find-the-angle-between-three-points/
	acos(euclideanDistance() * euclideanDistance())

	tienstraMethod(pos_a, pos_b, pos_c, );
}

bool on_parameter_changed(const Parameter* old_param, const Parameter* new_param, void* context)
{
	Serial.println("Param changed");
	return false;
}

#endif // ROS2_ENABLED
