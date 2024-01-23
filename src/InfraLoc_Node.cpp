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
	}
}

int createInfralocService()
{
	// Service server object
	rcl_service_t service;
	const char *service_name = "/beacon_angle";

	// Get message type support
	const rosidl_service_type_support_t *type_support =
		ROSIDL_GET_SRV_TYPE_SUPPORT(infraloc_interfaces, srv, BeaconAngle);

	// Initialize server with default configuration
	rcl_ret_t rc = rclc_service_init_default(
		&service, &node,
		type_support, service_name);

	if (rc != RCL_RET_OK)
	{
		return -1;
	}

	return RCL_RET_OK;
}

// Implementation example:
void service_callback(const void *request_msg, void *response_msg)
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

rcl_ret_t whatever2(rclc_support_t * support,
  int argc,
  char const * const * argv,
  rcl_init_options_t * init_options,
  rcl_allocator_t * allocator)
{

	RCL_CHECK_FOR_NULL_WITH_MSG(
		support, "support is a null pointer", return RCL_RET_INVALID_ARGUMENT);
	RCL_CHECK_FOR_NULL_WITH_MSG(
		init_options, "init_options is a null pointer", return RCL_RET_INVALID_ARGUMENT);
	RCL_CHECK_FOR_NULL_WITH_MSG(
		allocator, "allocator is a null pointer", return RCL_RET_INVALID_ARGUMENT);
	rcl_ret_t rc = RCL_RET_OK;

	support->context = rcl_get_zero_initialized_context();
	rc = rcl_init(argc, argv, init_options, &support->context);
	if (rc != RCL_RET_OK)
	{
		PRINT_RCLC_ERROR(rclc_init, rcl_init);
		return rc;
	}
	support->allocator = allocator;

	rc = rcl_clock_init(RCL_STEADY_TIME, &support->clock, support->allocator);
	if (rc != RCL_RET_OK)
	{
		PRINT_RCLC_ERROR(rclc_init, rcl_clock_init);
	}
	return rc;
}

rcl_ret_t whatever(rclc_support_t * support, int argc, char const * const * argv, rcl_allocator_t * allocator)
{
	RCL_CHECK_FOR_NULL_WITH_MSG(
		support, "support is a null pointer", return RCL_RET_INVALID_ARGUMENT);
	RCL_CHECK_FOR_NULL_WITH_MSG(
		allocator, "allocator is a null pointer", return RCL_RET_INVALID_ARGUMENT);
	rcl_ret_t rc = RCL_RET_OK;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rc = rcl_init_options_init(&init_options, (*allocator));
	if (rc != RCL_RET_OK)
	{
		PRINT_RCLC_ERROR(rclc_support_init, rcl_init_options_init);
		return rc;
	}

	rc = whatever2(support, argc, argv, &init_options, allocator);
	if (rcl_init_options_fini(&init_options) != RCL_RET_OK)
	{
		PRINT_RCLC_ERROR(rclc_support_init, rcl_init_options_fini);
	}

	return rc;
}


int init_infra_node()
{
	set_microros_serial_transports(Serial);

	allocator = rcl_get_default_allocator();

	//create init_options
	rcl_ret_t test = whatever(&support, 0, NULL, &allocator);
	if(test != RCL_RET_OK)
		Serial.print("ERR");

	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

	createInfralocService();

	return RCL_RET_OK;
}


int update_infra_node()
{
	// Spin executor to receive requests
	return rclc_executor_spin(&executor);
}
#endif // ROS2_ENABLED