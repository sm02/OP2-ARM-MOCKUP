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

/* Author: sm (2021) */

/* ROS API Header */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

std::string g_arm_ns;
std::string g_robot_ns;

sensor_msgs::JointState g_lastJointState;

ros::Publisher g_robot_pos_pub;

void positionCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::string n;
	n=msg->name[0];
	ROS_INFO("data : %s",n.c_str());
	g_lastJointState=*msg;
//	g_robot_pos_pub.publish(msg);		
}


void timerCallback(const ros::TimerEvent&) {
	g_robot_pos_pub.publish(g_lastJointState);	

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_driver");
  ros::NodeHandle nh;

  ROS_INFO("Arm driver init");
  
  /* Load ROS Parameter */
  nh.param<std::string>("arm_ns", g_arm_ns, "armmockup");
  nh.param<std::string>("robot_ns", g_robot_ns, "robotis");
  
  ros::Subscriber g_mockup_pos_sub = nh.subscribe("/"+g_arm_ns+"/present_joint_states", 1000, positionCallback);
  g_robot_pos_pub = nh.advertise<sensor_msgs::JointState>("/"+g_robot_ns+"/direct_control/set_joint_states", 0);
 
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);  
  
  while (ros::ok())
  {
    usleep(1 * 1000);

    ros::spin();
  }

  return 0;
}
