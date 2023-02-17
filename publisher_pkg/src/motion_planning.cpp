/**
 * @file motion_planning.cpp
 * @author Rigon Mattia (mattia.rigon@studenti.unitn.it)
 * @brief In this file there are the main function that permit to calculate the trajectory that the robot will do 
 * @version 0.1
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "publisher_pkg/motion_planning.h"
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

using namespace std;

/**
 * @brief Returns the rotation matrix corresponding to the given Euler angles
 * 
 * @param th Euler angles
 * @return ** RotationMatrix 
 */

RotationMatrix eul2rot(EulerVector th)
{
    RotationMatrix R;
    R << cos(th[0]) * cos(th[1]), cos(th[0]) * sin(th[1]) * sin(th[2]) - sin(th[0]) * cos(th[2]), cos(th[0]) * sin(th[1]) * cos(th[2]) + sin(th[0]) * sin(th[2]),
        sin(th[0]) * cos(th[1]), sin(th[0]) * sin(th[1]) * sin(th[2]) + cos(th[0]) * cos(th[2]), sin(th[0]) * sin(th[1]) * cos(th[2]) - cos(th[0]) * sin(th[2]),
        -sin(th[1]), cos(th[1]) * sin(th[2]), cos(th[1]) * cos(th[2]);

    return R;
}

/**
 * @brief Returns the euler vector associated to the R rotation Matrix
 * 
 * @param R RotationMatrix 3x3
 * @return ** EulerVector 
 */

EulerVector rot2eul(RotationMatrix R)
{

    double x = atan2(R(1, 0), R(1, 1));
    double y = atan2(-R(2, 0), sqrt(pow(R(2, 1), 2) + pow(R(2, 2), 2)));
    double z = atan2(R(2, 1), R(2, 2));

    EulerVector e(x, y, z);

    return e;
}

/**
 * @brief check for each angles if the ur5 can apply it on the correspondent joiny,checking for each angle if is inside the associate range in max_angles_value
 * 
 * @param Th Vector which contains values of all 6 joints angles
 * @return true 
 * @return false 
 */

bool check_angles(JointStateVector Th)
{

    int cont = 0;

    Eigen::Matrix<double, 6, 2> max_angles_value;
    max_angles_value << -6.14, 6.14,
        -3.14, 0,
        -3.14, 3.14,
        -6.28, 6.28,
        -6.28, 6.28,
        -6.28, 6.28;

    for (int i = 0; i < 6; i++)
    {

        if (Th(i) > max_angles_value(cont, 0) && Th(i) < max_angles_value(cont, 1))
        {
            cont = cont + 1;
            continue;
        }
        else
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Give the 6 joint angles, it performs the direct kinematics for each joint, so we can know the position of alle the 6 joint,then it check that
 *        this postion is inside some defined range. In our case we have to avoid the collision with 3 plans : 
 *        - the ceiling of our envoirment
 *        - the floor of our envoirment
 *        - the wall behind the robot
 * 
 * @param th1 
 * @param th2 
 * @param th3 
 * @param th4 
 * @param th5 
 * @param th6 
 * @return true 
 * @return false 
 */

bool check_singularity_collision(const double th1, const double th2, const double th3, const double th4, const double th5, const double th6)
{

    vector<TransformationMatrix> matrici;
    bool cond = true;
    float Pos_x, Pos_y, Pos_z;
    TransformationMatrix Tn;

    TransformationMatrix T10m = T10f(th1);
    matrici.push_back(T10m);
    TransformationMatrix T21m = T21f(th2);
    matrici.push_back(T21m);
    TransformationMatrix T32m = T32f(th3);
    matrici.push_back(T32m);
    TransformationMatrix T43m = T43f(th4);
    matrici.push_back(T43m);
    TransformationMatrix T54m = T54f(th5);
    matrici.push_back(T54m);
    TransformationMatrix T65m = T65f(th6);
    matrici.push_back(T65m);

    Tn << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    for (int i = 0; i < 6; i++)
    {

        Tn = Tn * matrici[i];
        Pos_x = Tn(0, 3);
        Pos_y = Tn(1, 3);
        Pos_z = Tn(2, 3);

        if (Pos_z < 0 or Pos_z > 0.745 + 0.14 or Pos_y > 0.25)
        {
            cout << Pos_z << " " << Pos_y << endl;
            cond = false;
            break;
        }
    }
    return cond;
}

/**
 * @brief It find the closest conifguration to the starting conifguration, which furthermore fulfills two conditions : 
 *        - the configuration must respect for each joint the limits it has
 *        - the configuration must not have any collision with singularities
 * 
 * @param qEs intital joints configuration
 * @param configurations all the 8 possible configurations , generated by the inverse kinematics
 * @return ** JointStateVector 
 */

JointStateVector nearest(JointStateVector qEs, vector<JointStateVector> configurations)
{


    double min_dist = 100000;
    int cont = 0;
    int pos = 0;
    JointStateVector e;
    double e_norm;
    for (JointStateVector conf : configurations)
    {

        if (!check_angles(conf))
        {
            cont = cont + 1;
            continue;
        }
        if (!check_singularity_collision(conf[0], conf[1], conf[2], conf[3], conf[4], conf[5]))
        {
            cont = cont + 1;
            continue;
        }
        e = conf - qEs;
        e_norm = e.norm();
        if (e_norm < min_dist)
        {
            min_dist = e_norm;
            pos = cont;
        }
        cont = cont + 1;
    }

    JointStateVector qEf = configurations[pos];

    return qEf;
}

/**
 * @brief Returns starting from tStart momement the trajectory to college togheter qEs e qEf
 * 
 * @param tStart t0 time
 * @param qEs initial joints configuration
 * @param qEf final desidered joints configuration
 * @param dt time interval between a configuration n and n+1
 * @param DtP time period that occur for connect qEs to qEf
 * @param DtA time period that each joint use for accelerate. Raccomanded value 1/4 DtP, MAX VALUE 1/2 DtP
 * @return vector<vector<double>> matrix , where each row is an intemediate configuration that it will reach dt time after the row befor 
 */

vector<vector<double>> thetaConnect2Points(const double tStart, JointStateVector qEs, JointStateVector qEf, const double dt, const double DtP, const double DtA)
{
    // route contains 6 vectors (pairs) , each of it contains start and end of q of the joint
    vector<vector<double>> route; 
    vector<double> couple(2);
    for (int i = 0; i < 6; i++)
    {
        couple.clear();
        couple.push_back(qEs[i]);
        couple.push_back(qEf[i]);
        route.push_back(couple);
    }

    // INITIAL ACCELERATION
    int size = (int)(DtA / dt);
    vector<double> T;
    for (double i = 0; i < DtA + dt; i = i + dt)
    {
        T.push_back(i);
    }
    vector<vector<double>> th;

    vector<double> last_q(7);

    vector<double> qp; // t + 6 joint 
    double vdes;
    double A;
    double q;

    for (double t : T)
    {
        qp.clear();
        qp.push_back(t + tStart);

        for (vector<double> joint : route)
        {
            vdes = (DtP / (DtP - DtA)) * (joint[1] - joint[0]) / (DtP);
            A = vdes / DtA;
            q = joint[0] + A * (pow(t, 2)) / 2;
            qp.push_back(q); // I add the positions of all the joints for instant t
        }

        th.push_back(qp); // I add the conf of the joints in t

        last_q = qp;
    }

    // I make the uniform motion from DtA , to DtP-DtA

    size = (int)((DtP - 2 * DtA) / dt);
    T.resize(size);
    T.clear();
    for (double i = DtA + dt; i < (DtP - DtA) + dt; i = i + dt)
    {
        T.push_back(i);
    }
    int pos;
    for (double t : T)
    {
        qp.clear();
        qp.push_back(t + tStart);
        pos = 1;

        for (vector<double> joint : route)
        {

            vdes = (DtP / (DtP - DtA)) * (joint[1] - joint[0]) / (DtP);
            q = last_q[pos] + vdes * (t - DtA);
            qp.push_back(q); 
            pos++;
        }
        th.push_back(qp); 
    }

    last_q = qp;

    // FINAL DECELERATION

    size = (int)((DtA) / dt);
    T.resize(size);
    T.clear();
    for (double i = DtP - DtA + dt; i <= DtP + dt; i = i + dt)
    {
        T.push_back(i);
    }

    for (double t : T)
    {
        qp.clear();
        qp.push_back(t + tStart);
        pos = 1;
        for (vector<double> joint : route)
        {
            vdes = (DtP / (DtP - DtA)) * (joint[1] - joint[0]) / (DtP);
            A = -vdes / DtA;
            q = last_q[pos] + vdes * (t - (DtP - DtA)) + A * (pow(t - (DtP - DtA), 2)) / 2;
            qp.push_back(q); 
            pos = pos + 1;
        }
        th.push_back(qp); 
    }

    return th;
}

/**
 * @brief starting from qEs configurations, calculate all the intermediate configurations (that will change every dt), draw a trajector that 
 *        passes through all the intermediate points, and make reach the xEf position (with end effecor) with phiEf final configurations.
 * 
 * @param qEs initial joint configurations
 * @param xEf final position that we want to reach
 * @param phiEf final euler angles that we want to reach
 * @param intermediate_points vector of intermediate position that we want to cross 
 * @param dt time interval between a configuration n and n+1
 * @param turn bool value that decide if this movement will end with the end effector turned 
 * @return ** vector<vector<double>> 
 */

vector<vector<double>> p2pMotionPlanIntermediatePoints(const JointStateVector qEs, const PositionVector xEf, EulerVector phiEf, vector<PositionVector> intermediate_points, double dt ,bool turn)
{

    vector<JointStateVector> qAll;
    qAll.push_back(qEs);

    JointStateVector qInt;
    JointStateVector last_q = qEs;
    EulerVector e(0, 0, 0);

    int i = 0;
    for (PositionVector item : intermediate_points)
    {   
        // if is a turn movement , the second to last point to reach will have alrady the final configuration phiEf
        if(i == intermediate_points.size() -1 && turn){
            item(2) = 0.6;
            qInt = nearest(last_q, inverse_kinematics(item, eul2rot(phiEf)));

        }else{
            qInt = nearest(last_q, inverse_kinematics(item, eul2rot(e)));
        }

        if(qInt[0] > M_PI/2){
            qInt[0] = qInt[0] - 2*M_PI;
        }
        qAll.push_back(qInt);
        last_q = qInt;
        i++;
    }

    // Optimizes excessive gripper rotations 
    if(phiEf(0) >  1.5 * M_PI){
        phiEf(0) = phiEf(0) - 2*M_PI;
    }
    if(phiEf(0) < - 1.5 * M_PI){
        phiEf(0) = phiEf(0) + 2*M_PI;
    }
     // I add final position
    qInt = nearest(last_q, inverse_kinematics(xEf, eul2rot(phiEf)));
    if(qInt[0] > M_PI/2){
        qInt[0] =qInt[0] - 2*M_PI;
    }
    qAll.push_back(qInt);

    vector<vector<double>> th;

    double DtA = 0.5; // time it takes starting from 0 to reach the final speed, or the other way around
    double DtP = 2;   // time it takes to get from one point to another

    int size = (int)qAll.size();
    vector<double> T(size);
    int incr = 0;
    iota(T.begin(), T.end(), incr);

    for (int i : T)
    {

        if (i + 1 < size) // I check whether it has a subsequent
        { 
            vector<vector<double>> res = thetaConnect2Points(i * DtP, qAll[i], qAll[i + 1], dt, DtP, DtA);
            th.insert(th.end(), res.begin(), res.end());
        }
    }

    return th;
}