/*******************************************************************************
* Copyright 2021
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman, sm (2021) */

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
//#include "op3_action_module/action_module.h"
//#include "op3_direct_control_module/direct_control_module.h"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;

const int BAUD_RATE = 57600;
const double PROTOCOL_VERSION = 1.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;
const int TORQUE_ON_CTRL_TABLE = 64;

bool g_is_simulation = false;
int g_baudrate;
std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;

ros::Publisher g_dxl_torque_pub;
ros::Publisher g_control_mode_pub;
ros::Publisher sync_write_pub;

void dxlTorqueCheckCallback(const std_msgs::String::ConstPtr& msg)
{
  if (g_is_simulation == true)
    return;

  if (msg->data == "off") {
		ROS_INFO("dxl_torque off received");
		robotis_controller_msgs::SyncWriteItem syncwrite_msg;
		int torque_value = 0;
		syncwrite_msg.item_name = "torque_enable";

		syncwrite_msg.joint_name.push_back("l_sho_pitch");
		syncwrite_msg.value.push_back(torque_value);
		syncwrite_msg.joint_name.push_back("l_sho_roll");
		syncwrite_msg.value.push_back(torque_value);
		syncwrite_msg.joint_name.push_back("l_el");
		syncwrite_msg.value.push_back(torque_value);

		sync_write_pub.publish(syncwrite_msg);
  } 
  else
  {
		ROS_INFO("dxl_torque check and set on");
	  // check dxl torque
	  uint8_t torque_result = 0;
	  bool torque_on = true;
	  RobotisController *controller = RobotisController::getInstance();

	  for (std::map<std::string, std::string>::iterator map_it = controller->robot_->port_default_device_.begin();
		   map_it != controller->robot_->port_default_device_.end(); map_it++)
	  {
		std::string default_device_name = map_it->second;
		controller->read1Byte(default_device_name, TORQUE_ON_CTRL_TABLE, &torque_result);

		// if not, torque on
		if (torque_result != 1)
		  torque_on = false;
	  }

	  if(torque_on == false)
	  {
		controller->stopTimer();

		controller->initializeDevice(g_init_file);

		controller->startTimer();
	  }
  }
}

void timerCallback(const ros::TimerEvent&) {
  // no torque
  std_msgs::String dxl_torque_msg;
  dxl_torque_msg.data = "off";

  g_dxl_torque_pub.publish(dxl_torque_msg);
  ROS_INFO("dxl torque off");	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "armmockup_manager");
  ros::NodeHandle nh;

  ROS_INFO("manager->init");
  RobotisController *controller = RobotisController::getInstance();
  //controller->DEBUG_PRINT=true;
  /* Load ROS Parameter */

  nh.param<std::string>("offset_file_path", g_offset_file, "");
  nh.param<std::string>("robot_file_path", g_robot_file, "");
  nh.param<std::string>("init_file_path", g_init_file, "");
  nh.param<std::string>("device_name", g_device_name, SUB_CONTROLLER_DEVICE);
  nh.param<int>("baud_rate", g_baudrate, BAUD_RATE);

  ros::Subscriber dxl_torque_sub = nh.subscribe("/robotis/dxl_torque", 1, dxlTorqueCheckCallback);
  g_dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  
  g_control_mode_pub = nh.advertise<std_msgs::String>("/robotis/set_control_mode", 0);  
  sync_write_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  nh.param<bool>("gazebo", controller->gazebo_mode_, false);
  g_is_simulation = controller->gazebo_mode_;

  if (g_is_simulation) /* gazebo simulation */
  {
    ROS_WARN("SET TO GAZEBO MODE!");
    std::string robot_name;
    nh.param<std::string>("gazebo_robot_name", robot_name, "");
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }


  if (g_robot_file == "")
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);

  usleep(300 * 1000);

  /* Add Motion Module */
//  controller->addMotionModule((MotionModule*) ActionModule::getInstance());
  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
//  controller->addMotionModule((MotionModule*) DirectControlModule::getInstance());

  // start timer
  controller->startTimer();

  usleep(100 * 1000);

//Error "Bulk Read" with MotionControlMode, leading to present_joint_state not updated.
//This control mode is not usefull, so switch to direct control mode by default.
  std_msgs::String direct_control_msg;
  direct_control_msg.data = "DirectControlMode";

  g_control_mode_pub.publish(direct_control_msg);
  ROS_INFO("Direct Control mode enabled");

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback, true);  // oneshot
 


  while (ros::ok())
  {
    usleep(1 * 1000);

    ros::spin();
  }

  return 0;
}
