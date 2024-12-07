//============================================================================
// Name        : maxon_epos2_node.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Node for maxon_epos2, initialization of ROS.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "rclcpp/rclcpp.hpp"
#include "maxon_driver/EposController.hpp"

int main(int argc, char** argv)
{
	/*
	 * We have a publisher, so we can not use ros::spin(), but spinOnce and sleep in a while loop instead.
	 * spinOnce is needed for subscriber callback.
	 */
	void* pKeyHandle = 0;
	rclcpp::init(argc, argv);
	std::shared_ptr<maxon_epos2::EposController> node2 = std::make_shared<maxon_epos2::EposController>(2, "pitch", 367., -16000, 0, &pKeyHandle);
	std::shared_ptr<maxon_epos2::EposController> node1 = std::make_shared<maxon_epos2::EposController>(1, "roll", 100., 9000, 1, &pKeyHandle);
	// rclcpp::Rate loop_rate(100);
	rclcpp::executors::StaticSingleThreadedExecutor executor;
	executor.add_node(node1);
	executor.add_node(node2);
	executor.spin();

	//publish until node gets interrupted
	// rclcpp::spin(node2);
	// rclcpp::spin(node1);
	
	//if node is interrupted, close device
	node1->close_device(0.);
	node2->close_device(70.);
	

	return 0;
}
