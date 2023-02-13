
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

RotationMatrix eul2rot(EulerVector th)
{
    /*
    Returns the rotation matrix
    th[0] = roll , th[1] = pitch ,th[2] = yaw
    */
    

    RotationMatrix R;
    R << cos(th[0]) * cos(th[1]), cos(th[0]) * sin(th[1]) * sin(th[2]) - sin(th[0]) * cos(th[2]), cos(th[0]) * sin(th[1]) * cos(th[2]) + sin(th[0]) * sin(th[2]),
        sin(th[0]) * cos(th[1]), sin(th[0]) * sin(th[1]) * sin(th[2]) + cos(th[0]) * cos(th[2]), sin(th[0]) * sin(th[1]) * cos(th[2]) - cos(th[0]) * sin(th[2]),
        -sin(th[1]), cos(th[1]) * sin(th[2]), cos(th[1]) * cos(th[2]);

    return R;
}

EulerVector rot2eul(RotationMatrix R)
{

    // Returns the euler vector associated to the R rotation Matrix

    double x = atan2(R(1, 0), R(1, 1));
    double y = atan2(-R(2, 0), sqrt(pow(R(2, 1), 2) + pow(R(2, 2), 2)));
    double z = atan2(R(2, 1), R(2, 2));

    EulerVector e(x, y, z);

    return e;
}

bool check_angles(JointStateVecor Th)
{

    // check for each angles if the ur5 can apply it . Checking for each angle if is inside the associate range in max_angles_value

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
bool check_singularity_collision(const double th1, const double th2, const double th3, const double th4, const double th5, const double th6)
{

    vector<TransformationMatrix> matrici;

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

    TransformationMatrix Tn;
    Tn << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    bool cond = true;
    float Pos_x, Pos_y, Pos_z;

    // calcolo la posizione dell n-esimo end effector
    // Per selezionare il numero di end-effector modificare la variabile num_joint
    for (int i = 0; i < 6; i++)
    {

        Tn = Tn * matrici[i];
        Pos_x = Tn(0, 3);
        Pos_y = Tn(1, 3);
        Pos_z = Tn(2, 3);

<<<<<<< HEAD
        if (Pos_z < 0 or Pos_z > 0.745 - 0.14 or Pos_y > 0.25)
=======
        if (Pos_z < 0 or Pos_z > 0.745 + 0.15 or Pos_y > 0.25)
>>>>>>> 8111b3c94a7b08080b1a9d83a844035f14ccc9f6
        {
            //cout << Pos_z << " " << Pos_y << endl;
            cond = false;
            break;
        }
    }
    return cond;
}

JointStateVecor nearest(JointStateVecor qEs, vector<JointStateVecor> configurations)
{

    // trovo la conifgurazione + vicina a quella di partenza

    double min_dist = 100000;
    int cont = 0;
    int pos = 0;
    JointStateVecor e;
    double e_norm;
    for (JointStateVecor conf : configurations)
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

    JointStateVecor qEf = configurations[pos];

    return qEf;
}

vector<vector<double>> thetaConnect2Points(const double tStart, JointStateVecor qEs, JointStateVecor qEf, const double dt, const double DtP, const double DtA)
{

    // Returns starting from tStart momement the trajectory to college togheter qEs e qEf
    vector<vector<double>> route; // route contiene 6 vettori (coppie) , ognuna di essa contiene inizio e fine di q del joint
    vector<double> couple(2);
    for (int i = 0; i < 6; i++)
    {
        couple.clear();
        couple.push_back(qEs[i]);
        couple.push_back(qEf[i]);
        route.push_back(couple);
    }

    // ACCELLERAZIONE INIZIALE##
    int size = (int)(DtA / dt);
    vector<double> T;
    for (double i = 0; i < DtA + dt; i = i + dt)
    {
        T.push_back(i);
    }
    vector<vector<double>> th;

    vector<double> last_q(7);

    vector<double> qp; // t + 6 joint (forse si potrebbe cambiare tipo usando eigen boh)
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
            qp.push_back(q); // aggiungo le posizioni di tutti i joint per l'istante t
        }

        th.push_back(qp); // aggiungo la conf dei joint in t

        last_q = qp;
    }

    // faccio il moto uniforme da Dta , fino a DtP-DtA

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
            qp.push_back(q); // aggiungo le posizioni di tutti i joint per l'istante t
            pos++;
        }
        th.push_back(qp); // aggiungo la conf dei joint in t
    }

    last_q = qp;

    // faccio decellerare

    size = (int)((DtA) / dt);
    T.resize(size);
    T.clear();
    // n = DtP-DtA;
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
            qp.push_back(q); // aggiungo le posizioni di tutti i joint per l'istante t
            pos = pos + 1;
        }
        th.push_back(qp); // aggiungo la conf dei joint in t
    }

    return th;
}

vector<vector<double>> p2pMotionPlanIntermediatePoints(const JointStateVecor qEs, const PositionVecor xEf, const EulerVector phiEf, vector<PositionVecor> intermediate_points, double dt ,bool turn)
{

    // xEs, phiEs, xEf, phiEf,points,minT,maxT,
    //  parte da qEs(joints) e deve raggiungere poszione xEf con phiEf passando per tutti gli intermediate points
    vector<JointStateVecor> qAll;
    qAll.push_back(qEs);

    JointStateVecor qInt;
    JointStateVecor last_q = qEs;
    EulerVector e(0, 0, 0);

    int i = 0;
    for (PositionVecor item : intermediate_points)
    {
        if(i == intermediate_points.size() -1 && turn){
            item(2) = 0.6;
            qInt = nearest(last_q, inverse_kinematics(item, eul2rot(phiEf)));
            cout << "intermedio storto"<< endl;

        }else{
            qInt = nearest(last_q, inverse_kinematics(item, eul2rot(e)));
        }
        //qInt = nearest(last_q, inverse_kinematics(item, eul2rot(e)));


        if(qInt[0] > M_PI/2){
            qInt[0] =qInt[0] - 2*M_PI;
            cout << "cambiato angoli 1" << endl;
        }
        qAll.push_back(qInt);
        last_q = qInt;
        i++;
    }
    qInt = nearest(last_q, inverse_kinematics(xEf, eul2rot(phiEf)));
    if(qInt[0] > M_PI/2){
        qInt[0] =qInt[0] - 2*M_PI;
        cout << "cambiato angoli 2" << endl;

    }
    qAll.push_back(qInt);

    vector<vector<double>> th;

    double DtA = 0.5; // tempo che ci mette partendo da 0 a raggiungere la velocità finale, o il contrario
    double DtP = 2;   // tempo che ci mette per andare da un punto all'altro

    int size = (int)qAll.size();
    vector<double> T(size);
    int incr = 0;
    iota(T.begin(), T.end(), incr);

    for (int i : T)
    {

        if (i + 1 < size) // controllo se ha un successivo
        { 
            vector<vector<double>> res = thetaConnect2Points(i * DtP, qAll[i], qAll[i + 1], dt, DtP, DtA);
            th.insert(th.end(), res.begin(), res.end());
        }
    }

    return th;
}