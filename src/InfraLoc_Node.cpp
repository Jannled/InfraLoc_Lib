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

int InfraNode::init()
{
	set_microros_serial_transports(Serial);

	allocator = rcl_get_default_allocator();

	//create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "infraloc", "", &support));

	// Executor init example with the minimum RCLC executor handles required
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(
		&executor, &support.context,
		RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator
	));

	createInfralocService();
	createStrengthMessage();

	return RCL_RET_OK;
}


int InfraNode::update()
{
	// Spin executor to receive requests
	return rclc_executor_spin_some(&executor, 1000000 * spinMillis);
}

int InfraNode::publishBucketStrength(std::array<number_t, INFRALOC_NUM_CHANNELS> values)
{
	infraloc_interfaces__msg__BucketStrength msg;
	for(size_t i=0; i<INFRALOC_NUM_CHANNELS; i++)
		msg.bucket_strength[i] = values.at(i);

	return rcl_publish(&strengthPublisher, &msg, NULL);
}

#endif // ROS2_ENABLED