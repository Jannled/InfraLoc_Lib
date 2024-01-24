#ifdef MICRO_ROS_ENABLED
#include "InfraLoc_Node.hpp"

#include <Arduino.h>

#include <micro_ros_platformio.h>

#include "infraloc_interfaces/srv/beacon_angle.h"

// Private variables
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error handle loop
void error_loop() {
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

int createInfralocService()
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

int initInfraNode()
{
	set_microros_serial_transports(Serial);

	allocator = rcl_get_default_allocator();

	//create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "infraloc", "", &support));

	createInfralocService();

	return RCL_RET_OK;
}


int updateInfraNode()
{
	// Spin executor to receive requests
	return rclc_executor_spin(&executor);
}

#endif // ROS2_ENABLED