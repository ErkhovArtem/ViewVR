//============================================================================
// Name        : EposController.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2
//				 (subscribers, parameters, timers, etc.).
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "maxon_driver/EposController.hpp"

namespace maxon_epos2 {

EposController::EposController(unsigned short Id, std::string name, float gear_ratio, int offset, bool IsSlave, void** p_pKeyHandle) : Node("maxon_node_" + name)
{

  //Initialize device:
  if((epos_device_.initialization(Id, gear_ratio, offset, IsSlave, p_pKeyHandle))==MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"Device initialization");
  //Start position mode during homing callback function:
  //if((epos_device_.startPositionMode())==MMC_FAILED) ROS_ERROR("Starting position mode failed");


//   publisher_ = this->create_publisher<maxon_interfaces::msg::Motor>("epos_info_topic", 10);
  subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "epos_motor_cmd_" + name, 1, std::bind(&EposController::subscriptionCallback, this, std::placeholders::_1));

  homing_service_ = this->create_service<std_srvs::srv::Trigger>("epos_homing_service_" + name, 
  	std::bind(&EposController::homingCallback, this, std::placeholders::_1, std::placeholders::_2));

//   service_ = this->create_service<maxon_interfaces::srv::Motor>("epos_control_service", 
//   std::bind(&EposController::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));


  RCLCPP_INFO(this->get_logger(),"Successfully launched EPOS Controller node.");
}

EposController::~EposController() {};


bool EposController::homingCallback(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response){
	(void)request;
	RCLCPP_INFO(this->get_logger(),"Requested homing service");
	//Home device:
	RCLCPP_INFO(this->get_logger(),"Homing...");
	if((epos_device_.homing())==MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"Device homing failed");
	else{
		//Start position mode:
		RCLCPP_INFO(this->get_logger(),"Start position mode");
		if((epos_device_.startPositionMode())==MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"Starting position mode failed");
		if(!epos_device_.setPosition(0.)) RCLCPP_ERROR(this->get_logger(),"set zero position failed");
		response->success = MMC_SUCCESS;
	}
	return true;
}

// bool EposController::serviceCallback(const std::shared_ptr<maxon_interfaces::srv::Motor::Request> request, 
// 	std::shared_ptr<maxon_interfaces::srv::Motor::Response> response)
// 	{
// 	RCLCPP_INFO_STREAM(this->get_logger(),"Requested position" << request->position_setpoint);
// 	if(!epos_device_.setPosition(request->position_setpoint)) RCLCPP_ERROR(this->get_logger(),"setPosition failed");
// 	response->success = true;
// 	if((epos_device_.getPosition(&response->position)) == MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"getPosition failed for service");
// 	if((epos_device_.getVelocity(&response->velocity)) == MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"getVelocity failed for service");
// 	return true;
// }

void EposController::subscriptionCallback(const std_msgs::msg::Float32::SharedPtr msg)
	{
	RCLCPP_INFO_STREAM(this->get_logger(),"Requested position" << msg->data);
	if(!epos_device_.setPosition(msg->data)) RCLCPP_ERROR(this->get_logger(),"setPosition failed");
	// if((epos_device_.getPosition(&response->position)) == MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"getPosition failed for service");
	// if((epos_device_.getVelocity(&response->velocity)) == MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"getVelocity failed for service");
}

// void EposController::publisher_loop(){
// 	if((epos_device_.deviceOpenedCheck()) == MMC_SUCCESS)
// 	{

// 		epos_device_.getPosition(&motor.position);
// 		epos_device_.getVelocity(&motor.velocity);
// // ****only output these for DEBUGGING******
// 		if((epos_device_.getPosition(&motor.position)) == MMC_FAILED) ROS_ERROR("getPosition failed for message");
// 		if((epos_device_.getVelocity(&motor.velocity)) == MMC_FAILED) ROS_ERROR("getVelocity failed for message");
// 		publisher_->publish(motor);
// 	}
// 	else{
// 		//****only for DEBUGGING****
// //		ROS_INFO("Nothing to publish.");
// 	}
// }

void EposController::close_device(float pos){
	  if(!epos_device_.setPosition(pos)) RCLCPP_ERROR(this->get_logger(),"setPosition failed");
	  if(!epos_device_.waitForTargetReached()) RCLCPP_ERROR(this->get_logger(),"Initial pose not reached");
	  if((epos_device_.closeDevice()) == MMC_FAILED) RCLCPP_ERROR(this->get_logger(),"Device closing failed");
}

} /* namespace */
