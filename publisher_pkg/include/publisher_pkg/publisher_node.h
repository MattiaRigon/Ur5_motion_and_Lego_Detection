/**
 * @file publisher_node.h
 * @author Rigon Mattia (mattia.rigon@studenti.unitn.it)
 * @version 0.1
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __PUBLISHER__ 
#define __PUBLISHER__


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include "motion_planning.h"
#include <list>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <thread>

float UNIT_BLOCCHETTO = 0.0125;

//types
typedef Eigen::Matrix<double, 8, 1> JointStateGripperVector;
typedef Eigen::Matrix<double, 2, 1> GripperState;
typedef struct Quaternion {
    double w, x, y, z;
}Quaternion;

// Functions
EulerVector ToEulerAngles(Quaternion q);
void send_des_jstate(const vector<double> & joint_pos);
JointStateVector return_joint_states();
void move_to(PositionVector pos,EulerVector e ,ros::Rate rate,bool turn);
bool check_point(PositionVector _pos,EulerVector e );
void close_gripper();
void open_gripper();
GripperState return_gripper_states();
void listen_lego_detection_turn(ros::Rate rate);

// Variables
GripperState actual_gripper;
double loop_time = 0.;
double loop_frequency = 1000.;
float position;
bool first_msg = false;
bool real_robot = true;  // true : puoi comandare solamente i 6 giunti e il gripper per lui non esiste
                         // fasle : puoi comandare anche il gripper (soft) , quindi pubblica comandi di dimensione 8
bool soft_gripper = true ;


// Publishers
ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;
std_msgs::Float64MultiArray jointState_msg_robot;

#endif