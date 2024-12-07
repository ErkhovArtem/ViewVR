//============================================================================
// Name        : EposController.hpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#pragma once

// STD
#include <string>
#include <chrono>
#include <functional>
#include <memory>

#include "maxon_driver/EposCommunication.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"

#include "maxon_interfaces/msg/motor.hpp"
#include "maxon_interfaces/srv/motor.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>


namespace maxon_epos2 {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class EposController: public rclcpp::Node
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  EposController(unsigned short Id, std::string name, float gear_ratio, int offset, bool IsSlave, void** p_pKeyHandle);

  /*!
   * Destructor.
   */

  virtual ~EposController();

  // void publisher_loop();
  void close_device(float pos);

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool homingCallback(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void subscriptionCallback(const std_msgs::msg::Float32::SharedPtr msg);
  // bool serviceCallback(const std::shared_ptr<maxon_interfaces::srv::Motor::Request> request, std::shared_ptr<maxon_interfaces::srv::Motor::Response> response);


  //! ROS topic publisher.
  // rclcpp::Publisher<maxon_interfaces::msg::Motor>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;

  //! ROS service server
  // rclcpp::Service<maxon_interfaces::srv::Motor>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service_;

  //! Device object
  EposCommunication epos_device_;

  //! Create variable for publishing motor info
  maxon_interfaces::msg::Motor motor;
};

} /* namespace */
