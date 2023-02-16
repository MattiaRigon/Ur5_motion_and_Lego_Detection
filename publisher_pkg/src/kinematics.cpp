#include <iostream>
#include "publisher_pkg/kinematics.h" 
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

using namespace std;

/**
 * @brief Give th1 returns the transformation matrix from frame 0 to frame 1
 * 
 * @param th1 
 * @return ** TransformationMatrix 
 */
TransformationMatrix T10f(const double th1){

    TransformationMatrix T10m;
    T10m << cos(th1), -sin(th1), 0, 0,
        sin(th1), cos(th1), 0, 0,
        0, 0, 1, D[0],
        0, 0, 0, 1;   
    return T10m;

}

/**
 * @brief Give th2 returns the transformation matrix from frame 1 to frame 2
 * 
 * @param th2 
 * @return ** TransformationMatrix 
 */

TransformationMatrix T21f(const double th2){

    TransformationMatrix T21m;
    T21m << cos(th2), -sin(th2), 0, 0,
            0, 0, -1, 0,
            sin(th2), cos(th2), 0, 0,
            0, 0, 0, 1;
    return T21m;

}


/**
 * @brief Give th3 returns the transformation matrix from frame 2 to frame 3
 * 
 * @param th3 
 * @return ** TransformationMatrix 
 */

TransformationMatrix T32f(const double th3){

    TransformationMatrix T32m;
    T32m << cos(th3), -sin(th3), 0, A[1],
            sin(th3), cos(th3), 0, 0,
            0, 0, 1, D[2],
            0, 0, 0, 1;    
    return T32m;

}

/**
 * @brief Give th4 returns the transformation matrix from frame 3 to frame 4
 * 
 * @param th4 
 * @return ** TransformationMatrix 
 */

TransformationMatrix T43f(const double th4){

    TransformationMatrix T43m;
    T43m << cos(th4), -sin(th4), 0, A[2],
            sin(th4), cos(th4), 0, 0,
            0, 0, 1, D[3],
            0, 0, 0, 1;
    return T43m;

}

/**
 * @brief Give th5 returns the transformation matrix from frame 4 to frame 5
 * 
 * @param th5 
 * @return ** TransformationMatrix 
 */

TransformationMatrix T54f(const double th5){

    TransformationMatrix T54m;
    T54m << cos(th5), -sin(th5), 0, 0,
            0, 0, -1, -D[4],
            sin(th5), cos(th5), 0, 0,
            0, 0, 0, 1;
    return T54m;

}

/**
 * @brief Give th6 returns the transformation matrix from frame 5 to frame 6
 * 
 * @param th6 
 * @return ** TransformationMatrix 
 */

TransformationMatrix T65f(const double th6){

    TransformationMatrix T65m;
    T65m << cos(th6), -sin(th6), 0, 0,
            0, 0, 1, D[5],
            -sin(th6), -cos(th6), 0, 0,
            0, 0, 0, 1;
    return T65m;

}

/**
 * @brief Transform from the joint angles space to the operational space, which is specified in terms of position and orientation of the end-effector  
 * 
 * @param th1 
 * @param th2 
 * @param th3 
 * @param th4 
 * @param th5 
 * @param th6 
 * @return ** DirectResult 
 */

DirectResult direct_kinematics(const double th1,const double th2,const double th3,const double th4,const double th5,const double th6){

    
    TransformationMatrix T10m = T10f(th1);
    TransformationMatrix T21m = T21f(th2);
    TransformationMatrix T32m = T32f(th3);
    TransformationMatrix T43m = T43f(th4);
    TransformationMatrix T54m = T54f(th5);
    TransformationMatrix T65m = T65f(th6);
    TransformationMatrix T06m ;

    T06m =T10m*T21m*T32m*T43m*T54m*T65m;
    PositionVector pos(*(T06m.data() +12),*(T06m.data() +13),*(T06m.data() +14));
    RotationMatrix rotMatrix = T06m.block<3,3>(0, 0);

    DirectResult result(pos,rotMatrix);

    return result;

}

/**
 * @brief Transform from the operational space, which is specified in terms of position and orientation of the end-effector , to the joint angles space
 * 
 * @param pos 
 * @param RotationMatrix 
 * @return ** vector<JointStateVector> 
 */

vector<JointStateVector> inverse_kinematics(PositionVector pos,RotationMatrix RotationMatrix){


    TransformationMatrix T60 ;
    T60.block<3,3>(0,0)=RotationMatrix;
    T60(0,3)=pos[0];
    T60(1,3)=pos[1];
    T60(2,3)=pos[2];
    T60(3,1)=0;
    T60(3,2)=0;
    T60(3,3)=1;

    Eigen::Matrix<double, 4, 1> p50;
    Eigen::Matrix<double, 4, 1> tmp = {{0},{0},{-D[5]},{1}};
    p50=T60*tmp;

    // Finding th1
    double th1_1 = double(atan2(p50[1], p50[0]) + acos(D[3]/hypot(p50[1], p50[0])))+M_PI/2;
    double th1_2 = double(atan2(p50[1], p50[0]) - acos(D[3]/hypot(p50[1], p50[0])))+M_PI/2;

    //finding th5
    double th5_1 = +double(acos((pos[0]*sin(th1_1) - pos[1]*cos(th1_1)-D[3]) / D[5]));
    double th5_2 = -double(acos((pos[0]*sin(th1_1) - pos[1]*cos(th1_1)-D[3]) / D[5]));
    double th5_3 = +double(acos((pos[0]*sin(th1_2) - pos[1]*cos(th1_2)-D[3]) / D[5]));
    double th5_4 = -double(acos((pos[0]*sin(th1_2) - pos[1]*cos(th1_2)-D[3]) / D[5]));
    

    //finding th6
    TransformationMatrix T06 ;
    T06 = T60.inverse();

    Eigen::Matrix<double, 3, 1> Xhat = T06.block<3,1>(0,0); 
    Eigen::Matrix<double, 3, 1> Yhat = T06.block<3,1>(0,1);

    double th6_1 = double(atan2(((-Xhat[1]*sin(th1_1)+Yhat[1]*cos(th1_1)))/sin(th5_1), ((Xhat[0]*sin(th1_1)-Yhat[0]*cos(th1_1)))/sin(th5_1)));
    //related to th11 a th52
    double th6_2 = double(atan2(((-Xhat[1]*sin(th1_1)+Yhat[1]*cos(th1_1))/sin(th5_2)), ((Xhat[0]*sin(th1_1)-Yhat[0]*cos(th1_1))/sin(th5_2))));
    //related to th12 a th53
    double th6_3 = double(atan2(((-Xhat[1]*sin(th1_2)+Yhat[1]*cos(th1_2))/sin(th5_3)), ((Xhat[0]*sin(th1_2)-Yhat[0]*cos(th1_2))/sin(th5_3))));
    //related to th12 a th54
    double th6_4 = double(atan2(((-Xhat[1]*sin(th1_2)+Yhat[1]*cos(th1_2))/sin(th5_4)), ((Xhat[0]*sin(th1_2)-Yhat[0]*cos(th1_2))/sin(th5_4))));
    

    Eigen::Matrix4d T41m ;
    T41m = (T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
    Eigen::Matrix<double, 3, 1> p41_1 = T41m.block<3,1>(0,3);
    double p41xz_1 = hypot(p41_1[0], p41_1[2]);

    T41m = (T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
    Eigen::Matrix<double, 3, 1> p41_2 = T41m.block<3,1>(0,3);
    double p41xz_2 = hypot(p41_2[0], p41_2[2]);
    
    T41m = (T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
    Eigen::Matrix<double, 3, 1> p41_3 = T41m.block<3,1>(0,3);
    double p41xz_3 = hypot(p41_3[0], p41_3[2]);
    
    T41m = (T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
    Eigen::Matrix<double, 3, 1> p41_4 = T41m.block<3,1>(0,3);
    double p41xz_4 = hypot(p41_4[0], p41_4[2]);

    //Computation of the 8 possible values for th3    
    double th3_1 = double(acos((p41xz_1*p41xz_1-A[1]*A[1]-A[2]*A[2])/(2*A[1]*A[2])));
    double th3_2 = double(acos((p41xz_2*p41xz_2-A[1]*A[1]-A[2]*A[2])/(2*A[1]*A[2])));
    double th3_3 = double(acos((p41xz_3*p41xz_3-A[1]*A[1]-A[2]*A[2])/(2*A[1]*A[2])));
    double th3_4 = double(acos((p41xz_4*p41xz_4-A[1]*A[1]-A[2]*A[2])/(2*A[1]*A[2])));
    
    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;
    
    //Computation of eight possible value for th2
    double th2_1 = double(atan2(-p41_1[2], -p41_1[0])-asin((-A[2]*sin(th3_1))/p41xz_1));
    double th2_2 = double(atan2(-p41_2[2], -p41_2[0])-asin((-A[2]*sin(th3_2))/p41xz_2));
    double th2_3 = double(atan2(-p41_3[2], -p41_3[0])-asin((-A[2]*sin(th3_3))/p41xz_3));
    double th2_4 = double(atan2(-p41_4[2], -p41_4[0])-asin((-A[2]*sin(th3_4))/p41xz_4));
    
    double th2_5 = double(atan2(-p41_1[2], -p41_1[0])-asin((A[2]*sin(th3_1))/p41xz_1));
    double th2_6 = double(atan2(-p41_2[2], -p41_2[0])-asin((A[2]*sin(th3_2))/p41xz_2));
    double th2_7 = double(atan2(-p41_3[2], -p41_3[0])-asin((A[2]*sin(th3_3))/p41xz_3));
    double th2_8 = double(atan2(-p41_4[2], -p41_4[0])-asin((A[2]*sin(th3_4))/p41xz_4));

    //finding th4
    Eigen::Matrix4d T43m ;
    Eigen::Matrix<double, 3, 1> Xhat43;
    T43m = (T32f(th3_1)).inverse()*(T21f(th2_1)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_1 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_2)).inverse()*(T21f(th2_2)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_2 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_3)).inverse()*(T21f(th2_3)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_3 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_4)).inverse()*(T21f(th2_4)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_4 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_5)).inverse()*(T21f(th2_5)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_5 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_6)).inverse()*(T21f(th2_6)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_6 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_7)).inverse()*(T21f(th2_7)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_7 = double(atan2(Xhat43[1], Xhat43[0]));
    
    T43m = (T32f(th3_8)).inverse()*(T21f(th2_8)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    double th4_8 = double(atan2(Xhat43[1], Xhat43[0]));

    // Load all the 8 solutions on Th

    vector<JointStateVector> Th;
    JointStateVector q;
     
    q << th1_1,th2_1,th3_1,th4_1,th5_1,th6_1;
    Th.push_back(q);
    q << th1_1,th2_2,th3_2,th4_2,th5_2,th6_2;
    Th.push_back(q);
    q << th1_2,th2_3,th3_3,th4_3,th5_3,th6_3;
    Th.push_back(q);
    q << th1_2,th2_4,th3_4,th4_4,th5_4,th6_4;
    Th.push_back(q);
    q << th1_1,th2_5,th3_5,th4_5,th5_1,th6_1;
    Th.push_back(q);
    q << th1_1,th2_6,th3_6,th4_6,th5_2,th6_2;
    Th.push_back(q);
    q << th1_2,th2_7,th3_7,th4_7,th5_3,th6_3;
    Th.push_back(q);
    q << th1_2,th2_8,th3_8,th4_8,th5_4,th6_4;
    Th.push_back(q);

    return Th;

}

