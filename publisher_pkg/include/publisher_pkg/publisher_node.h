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

//types
typedef Eigen::Matrix<double, 8, 1> JointStateVector;
typedef Eigen::Matrix<double, 2, 1> GripperState;
typedef struct Quaternion {
    double w, x, y, z;
}Quaternion;



// Methods
void send_des_jstate(const vector<double> & joint_pos);
JointStateVector secondOrderFilter(const JointStateVector & input, const double rate, const double settling_time);
bool check_point(PositionVecor _pos);

// Variables
GripperState actual_gripper;
double loop_time = 0.;
double loop_frequency = 1000.;
float position;
bool first_msg = false;
bool real_robot = false;


// Publishers
ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;
std_msgs::Float64MultiArray jointState_msg_robot;

#endif