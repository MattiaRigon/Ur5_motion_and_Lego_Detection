/**
 * @file motion_planning.h
 * @author Rigon Mattia (mattia.rigon@studenti.unitn.it)
 * @version 0.1
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __MOTION_PLANNING__ 
#define __MOTION_PLANNING__


#include "kinematics.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <list>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

RotationMatrix eul2rot(EulerVector th);
EulerVector rot2eul(RotationMatrix R);

bool check_singularity_collision(const double th1,const double th2,const double th3,const double th4,const double th5,const double th6);
bool check_angles(const JointStateVector Th);
JointStateVector nearest(JointStateVector qEs,vector<JointStateVector> configurations);
vector<vector<double>> thetaConnect2Points(const double tStart,JointStateVector qEs,JointStateVector qEf,const double dt,const double DtP,const double DtA);
vector<vector<double>> p2pMotionPlanIntermediatePoints(const JointStateVector qEs,const PositionVector xEf,const EulerVector phiEf,vector<PositionVector> intermediate_points,double dt,bool turn);


#endif
