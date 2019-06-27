/*
 *
 *encoder_to_wrist.cpp
 *
 *  Created on: Nov 16 2015
 *      Author: Camilo P
 *  Changed on: June 10, 2019
 *  Maintainer: lpetrich
 *
 */

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include "std_srvs/Empty.h"

#define PI        3.14159265358979323
#define CLOSE_GRASP 1.0
#define OPEN_GRASP  2.0
#define CLOSE_SPREAD  4.0
#define OPEN_SPREAD  8.0
#define MAXROTATION 2.2
#define OFFSET 1890.0
#define TICKS 4096.0


float roll_encoder_value, pitch_encoder_value, yaw_encoder_value, roll_rad_value, pitch_rad_value, yaw_rad_value, button_press;
// float OFFSET = 1890.0;
float j7 = 0.0;
ros::ServiceClient grasp_open_srv;
ros::ServiceClient grasp_close_srv;
ros::ServiceClient spread_open_srv;
ros::ServiceClient spread_close_srv;
bool open_grasp, close_grasp;
bool open_spread, close_spread;
float open_grasp_st, close_grasp_st;
float open_spread_st, close_spread_st;
std_srvs::Empty empty;

void wristCallback(const geometry_msgs::Quaternion::ConstPtr& wristMessage)
{
	open_grasp = close_grasp = open_spread = close_spread = false;
	roll_encoder_value = wristMessage->x;
	pitch_encoder_value = wristMessage->y;
	yaw_encoder_value = wristMessage->z;
	button_press = wristMessage->w;

	// std::cout << "roll_encoder_value: " << roll_encoder_value << std::endl;
	j7 = (roll_encoder_value - OFFSET) * 2.0 * PI / TICKS;
	// std::cout << "\tjoint 5: " << j7 << std::endl;
	if (j7 < -MAXROTATION) {
		j7 = -MAXROTATION;
		// std::cout << "\tTOO LOW joint 5: " << j7 << std::endl;
	} else if (j7 > MAXROTATION) {
		j7 = MAXROTATION; 
		// std::cout << "\tTOO HIGH joint 5: " << j7 << std::endl;
	} 
	// limit roll_joint
	// std::cout << "roll_encoder_value: " << roll_encoder_value << std::endl;
	// if (roll_encoder_value >= 0 && roll_encoder_value < 1024) {
	// 	roll_rad_value = (roll_encoder_value) * 2 * PI / 4096;
	// // } else if (roll_encoder_value >= 3072 && roll_encoder_value < 4096) {
	// } else {
	// 	std::cout << "roll_encoder_value: " << roll_encoder_value << std::endl;
	// 	roll_encoder_value = roll_encoder_value - 3072;
	// 	roll_encoder_value = 1024 - roll_encoder_value;
	// 	roll_rad_value = -(roll_encoder_value) * 2 * PI / 4096;
	// 	j5 = roll_rad_value;
	// 	std::cout << "\tjoint 5: " << j5 << std::endl;
	// 	if (j5 < -1.5) j5 = -1.5;
	// 	if (j5 > 1.5) j5 = 1.5;
	// }
	// limit pitch_joint
	// pitch_encoder_value = (pitch_encoder_value - 1410) * 2 * PI / 4096;
	// if (pitch_encoder_value < 1.50 && pitch_encoder_value > -1.50) {
	// 	pitch_rad_value = -pitch_encoder_value;
	// }
	// button_State_logic
 	// Gripper Grasp Command
	// Checking to see if gripper open button is being pressed
	if (button_press == OPEN_GRASP && (open_grasp_st != OPEN_GRASP)) {
		open_grasp = true; // set grasp publish state
	}
	//Checking to see if gripper close button is being pressed
	else if (button_press == CLOSE_GRASP && (close_grasp_st != CLOSE_GRASP)) {
		close_grasp = true; // set grasp publish state
	}
	open_grasp_st = button_press;
	close_grasp_st = button_press;
	// Gripper Spread Command
	if (button_press == OPEN_SPREAD && (open_spread_st != OPEN_SPREAD)) {
		open_spread = true;
	} else if (button_press == CLOSE_SPREAD && (close_spread_st != CLOSE_SPREAD)) {
		close_spread = true;
	}
	open_spread_st = button_press;
	close_spread_st = button_press;
	//////////////////
}



void update()
{

	if (open_grasp) {
		ROS_INFO("Grasp OPEN");
		grasp_open_srv.call(empty);
	} else if (close_grasp) {
		ROS_INFO("Grasp CLOSE");
		grasp_close_srv.call(empty);
	} else if (open_spread) {
		ROS_INFO("Spread OPEN");
		spread_open_srv.call(empty);
	} else if (close_spread) {
		ROS_INFO("Spread CLOSE");
		spread_close_srv.call(empty);
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "encoder_to_wrist");
	ros::NodeHandle nh;
//  ros::Subscriber jointSates_sub = nh.subscribe("/zeus/wam/joint_states", 1, wamJointStatesCallback);
	ros::Subscriber wrist_sub = nh.subscribe("/wrist", 1, wristCallback);
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/zeus/wam/jnt_hand_tool_cmd", 1);

	grasp_open_srv = nh.serviceClient<std_srvs::Empty>("zeus/bhand/open_grasp");
	grasp_close_srv = nh.serviceClient<std_srvs::Empty>("zeus/bhand/close_grasp");
	spread_open_srv = nh.serviceClient<std_srvs::Empty>("zeus/bhand/open_spread");
	spread_close_srv = nh.serviceClient<std_srvs::Empty>("zeus/bhand/close_spread");

	ros::Rate loop_rate(80);
	ROS_INFO("Main loop");
	//sensor_msgs::JointState send_joints;
	//send_joints.position.resize(7);
	while(ros::ok()) {
		sensor_msgs::JointState send_joints;
		//send_joints.position.resize(7);
		send_joints.position.push_back(0.0);
		send_joints.position.push_back(0.0);
		send_joints.position.push_back(0.0);
		send_joints.position.push_back(0.0);
		send_joints.position.push_back(0.0);
		send_joints.position.push_back(0.0);
		// send_joints.position.push_back(pitch_rad_value);
		send_joints.position.push_back(j7);
		// send_joints.position.push_back(roll_rad_value);
		joint_pub.publish(send_joints);
		update();
		ros::spinOnce();
		loop_rate.sleep();
		}
}
