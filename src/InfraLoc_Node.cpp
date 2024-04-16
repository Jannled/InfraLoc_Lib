#ifdef MICRO_ROS_ENABLED
#include "InfraLoc_Node.hpp"

#include <Arduino.h>

#include <micro_ros_platformio.h>

#include "rclc_parameter/rclc_parameter.h"
#include "infraloc_interfaces/msg/bucket_strength.h"
#include "infraloc_interfaces/srv/beacon_angle.h"


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
		RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator
	));

	// Create rcl state machine
	//state_machine = rcl_lifecycle_get_zero_initialized_state_machine();

	// Create the lifecycle node
	//rcl_ret_t rc = rclc_make_node_a_lifecycle_node(&my_lifecycle_node, &node, &state_machine, &allocator);

	// Register lifecycle services on the allocator
	//rclc_lifecycle_init_change_state_server(&my_lifecycle_node, &executor);
	//rclc_lifecycle_init_get_available_states_server(&my_lifecycle_node, &executor);
	//rclc_lifecycle_add_change_state_service(&my_lifecycle_node, &executor);

	// rclc_lifecycle_init_get_state_server
	// rclc_lifecycle_init_get_available_states_server
	// rclc_lifecycle_init_change_state_server

	createInfralocService();
	createStrengthMessage();

	createStrengthMessage2();
	createStrengthMessage3();

	return RCL_RET_OK;
}

int InfraNode::update()
{
	// For whatever reason, the uController enters HardFault when the executor spins
	//volatile rclc_executor_t test = executor;
	// Spin executor to receive requests
	//return rclc_executor_spin_some(&executor, 1000 * spinMillis);
	return 0;
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
	rclc_parameter_server_t param_server;

	// Initialize parameter server with default configuration
	rcl_ret_t rc = rclc_parameter_server_init_default(&param_server, &node);

	if(rc != RCL_RET_OK)
		return rc;

	// Add parameter to the server
  	rc = rclc_add_parameter(&param_server, "sample_frequency", RCLC_PARAMETER_INT);

	return rc;
}

int InfraNode::createInfralocService()
{
	// Service server object
	rcl_service_t service;
	const char* service_name = "/beacon_angle";

	// Get message type support
	const rosidl_service_type_support_t *type_support =
		ROSIDL_GET_SRV_TYPE_SUPPORT(infraloc_interfaces, srv, BeaconAngle);

	// Initialize server with default configuration
	rcl_ret_t rc = rclc_service_init_default(
		&service, &node,
		type_support, service_name);

	if(rc != RCL_RET_OK)
		return rc;

	// Service message objects
	infraloc_interfaces__srv__BeaconAngle_Response response_msg;
	infraloc_interfaces__srv__BeaconAngle_Request request_msg;

	// Add server callback to the executor
	rc = rclc_executor_add_service(&executor, &service, &request_msg,
		&response_msg, callback_beacon_angle
	);

	if(rc != RCL_RET_OK)
		return rc;

	return RCL_RET_OK;
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

#endif // ROS2_ENABLED
