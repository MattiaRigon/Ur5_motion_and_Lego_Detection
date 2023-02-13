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
bool check_angles(const JointStateVecor Th);
JointStateVecor nearest(JointStateVecor qEs,vector<JointStateVecor> configurations);
vector<vector<double>> thetaConnect2Points(const double tStart,JointStateVecor qEs,JointStateVecor qEf,const double dt,const double DtP,const double DtA);
vector<vector<double>> p2pMotionPlanIntermediatePoints(const JointStateVecor qEs,const PositionVecor xEf,const EulerVector phiEf,vector<PositionVecor> intermediate_points,double dt,bool turn);
bool check_singularity_collision(const double th1,const double th2,const double th3,const double th4,const double th5,const double th6);

#endif
