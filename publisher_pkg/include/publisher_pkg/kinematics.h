#ifndef __KINEMATICS__ 
#define __KINEMATICS__


#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

using namespace std;

const double A[6] = {0,-0.425,-0.3922,0,0, 0};
<<<<<<< HEAD
const double D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14};
=======
const double D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996 +0.14};
>>>>>>> 8111b3c94a7b08080b1a9d83a844035f14ccc9f6

typedef Eigen::Matrix<double, 1, 3> PositionVector;
typedef Eigen::Matrix<double, 1, 3> EulerVector;
typedef Eigen::Matrix4d TransformationMatrix;
typedef Eigen::Matrix3d RotationMatrix;
typedef Eigen::Matrix<double, 6, 1> JointStateVector;


typedef struct DirectResult{

    PositionVector pos;
    RotationMatrix rotMatrix;

    DirectResult(PositionVector _pos,RotationMatrix _rotMatrix){
        pos = _pos;
        rotMatrix = _rotMatrix; 
    }

}DirectResult;

TransformationMatrix T10f(const double th1);
TransformationMatrix T21f(const double th2);
TransformationMatrix T32f(const double th3);
TransformationMatrix T43f(const double th4);
TransformationMatrix T54f(const double th5);
TransformationMatrix T65f(const double th6);

DirectResult direct_kinematics(const double th1,const double th2,const double th3,const double th4,const double th5,const double th6);
vector<JointStateVector> inverse_kinematics(PositionVector pos,RotationMatrix RotationMatrix);

#endif