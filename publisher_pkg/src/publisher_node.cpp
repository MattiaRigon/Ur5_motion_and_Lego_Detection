/**
 * @file publisher_node.cpp
 * @author Rigon Mattia (mattia.rigon@studenti.unitn.it)
 * @brief This file is where it recive the position and orientation (of the lego) , and it publish on the right topic in order to 
 *        reach the reach the desidered position
 *        IMPORTANT : set properly the value of the real_robot variable in publisher_node.h
 * @version 0.1
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <list>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <thread>
#include "publisher_pkg/publisher_node.h"
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>
#include <spawnLego_pkg/legoDetection.h>
#include <spawnLego_pkg/legoGroup.h>
#include <ros_impedance_controller/generic_float.h>
#include <cmath>

using namespace std;


/**
 * @brief Convert from Quaternion to Euler Angles,this implementation assumes normalized quaternion
 * 
 * @param q Quaternion
 * @return ** EulerVector 
 */
EulerVector ToEulerAngles(Quaternion q) {
    EulerVector angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);


    angles << roll,pitch,yaw;
    return angles;
}

/**
 * @brief posts on the topic the vector joint_pos , which contains the value of the angles of all joints must reach
 * 
 * @param joint_pos vecort that conatins the values of each joint angle position that you're going to publish
 * @return ** void 
 */
void send_des_jstate(const JointStateGripperVector & joint_pos)
{

    if(real_robot){
        for (int i = 0; i < joint_pos.size() -2; i++)
        {
            jointState_msg_robot.data[i] = joint_pos[i];
        }
    }else{
        for (int i = 0; i < joint_pos.size() -2; i++)
        {
            jointState_msg_robot.data[i] = joint_pos[i];
        }
        for (int i=6;i<8;i++){
            jointState_msg_robot.data[i] = actual_gripper[i-6];
        }   
    }

    pub_des_jstate.publish(jointState_msg_robot);

}

/**
 * @brief Read from the topic the actual value of the joint 
 * 
 * @return ** JointStateVector 
 */
JointStateVector return_joint_states(){

    ros::NodeHandle node_1;
    ros::Duration Timeout = ros::Duration(3);

    JointStateVector actual_pos ;

    boost::shared_ptr<const sensor_msgs::JointState_<std::allocator<void>>> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states",node_1, Timeout );

    if(real_robot){
        actual_pos[0]= msg->position[2];
        actual_pos[1]= msg->position[1];
        actual_pos[2]= msg->position[0];
        actual_pos[3]= msg->position[3];
        actual_pos[4]= msg->position[4];
        actual_pos[5]= msg->position[5];
    }else{
        actual_pos[0]= msg->position[4];
        actual_pos[1]= msg->position[3];
        actual_pos[2]= msg->position[0];
        actual_pos[3]= msg->position[5];
        actual_pos[4]= msg->position[6];
        actual_pos[5]= msg->position[7];
    }

    return actual_pos;
}

/**
 * @brief Move the robot to the position pos, with e orientation 
 * 
 * @param pos position that we want to reach
 * @param e orientatin that we want to reach
 * @param rate ros rate
 * @param turn bool value that decide if this movement will end with the end effector turned 
 * @return ** void 
 */

void move_to(PositionVector pos,EulerVector e ,ros::Rate rate,bool turn){



    if(pos[0] == 0){
        pos[0]= 0.01;
    }
    if(pos[1] == 0){
        pos[1]= 0.01;
    }

    vector<PositionVector> intermediate;
    JointStateVector actual_pos = return_joint_states();
    DirectResult direct_res = direct_kinematics(actual_pos(0),actual_pos(1),actual_pos(2),actual_pos(3),actual_pos(4),actual_pos(5));

    PositionVector i1 ;
    i1 = direct_res.pos;
    i1(2)=0.5 + 0.14;
    intermediate.push_back(i1);

    PositionVector i ;
    i<< pos[0],pos[1],0.5+0.14;
    intermediate.push_back(i);

    double dt = 0.001;
    double DtP = 2;
    double DtA = 0.5;
    JointStateGripperVector pos_send;
    PositionVector pos_check;
    pos_send<<0,0,0,0,0,0,0,0;
    ros::Rate loop_rate(loop_frequency);
    auto res = p2pMotionPlanIntermediatePoints(actual_pos,pos,e,intermediate,0.001,turn);
    
    for (vector<double> conf : res){
        if(!check_singularity_collision(conf[1],conf[2],conf[3],conf[4],conf[5],conf[6])){
            cout << "COLLISON WITH SINGULARITY' "<<endl;
            cout <<"To go from : "<<direct_res.pos<<endl;
            cout << "to : "<< pos <<endl;

            return;
        }
    }

    for (vector<double> conf : res){

        for(int i=1;i<7;i++){

            pos_send(i-1)=conf[i];

        }
        send_des_jstate(pos_send);
        loop_rate.sleep();
    }

}


/**
 * @brief Listen to the /lego_position topic if messages arrives from the vision node , for each lego detected it send the robot to the lego position
 *        knowing which type of lego is , it can know which move it has to do, if it has to turn or not .
 * 
 * @param rate ros rate
 * @return ** void 
 */
void listen_lego_detection_turn(ros::Rate rate){

    ros::NodeHandle node_1;
    spawnLego_pkg::legoGroup::ConstPtr msg = ros::topic::waitForMessage<spawnLego_pkg::legoGroup>("/lego_position",node_1 );
    
    if(msg!=0){
        vector<spawnLego_pkg::legoDetection> vector = msg->lego_vector ;
        for (spawnLego_pkg::legoDetection lego : vector){

            cout << lego.model << endl;
            PositionVector pos;

            pos << lego.pose.position.x-0.5,-(lego.pose.position.y-0.35),-(lego.pose.position.z-1.75);
            Quaternion q ;
            q.x = lego.pose.orientation.x;
            q.y = lego.pose.orientation.y;
            q.z = lego.pose.orientation.z;
            q.w = lego.pose.orientation.w;

            EulerVector rot  = ToEulerAngles(q);
            EulerVector turn_rot; 
            cout << rot << endl;

            if(rot[0] == 0 && rot[1] == 0){ // blocchetto dritto 
                //z1 3.8
                //z2 5.8
                cout << "STRAIGHT LEGO" <<endl;
                int altezza = int(lego.model[7]) - 48; // se è in piedi è la z che da la sua altezza
                cout << "altezza : " <<altezza << endl;
                altezza = altezza * 0.01;  // o unit blocchetto bisogna vedere

                pos(2) = pos(2) - altezza;

                rot << -rot[2],0,0;

                if(check_point(pos,rot)){
                    cout <<" REACHABLE POSITION" <<endl;
                }else{
                    cout <<" NON REACHABLE POSITION "<<endl;
                    cout << pos << endl;
                    continue;
                }
                open_gripper();
                move_to(pos,rot,rate,false);
                             
                cout << endl ;
                cout << endl ;
                cout << endl ;
                
            }else if(rot[0] != 0 && rot[1] == 0){ // in piedi , solamente una rotazione

                cout << "TURNED LEGO (Y as height)" <<endl;

                int altezza = int(lego.model[4]) -48 ; // se è in piedi è la y che da la sua altezza
                float altezza_cm = (altezza - 1 ) * UNIT_BLOCCHETTO;
                cout << "tolgo altezza : " <<altezza_cm << endl;

                pos(2) = pos(2) - altezza_cm;
                rot << -rot[2],0,0;

                if(check_point(pos,rot)){
                    cout <<" REACHABLE POSITION" <<endl;
                }else{
                    cout <<" NON REACHABLE POSITION "<<endl;
                    cout << pos << endl;
                    continue;
                }

                open_gripper();
                move_to(pos,rot,rate,false);
                close_gripper();
                turn_rot << M_PI/2,0,-M_PI/2;
                pos(2) = 0.82;
                pos << -0.1 , -0.3 , 0.82;

                if(check_point(pos,turn_rot)){
                    cout <<" REACHABLE POSITION" <<endl;
                }else{
                    cout <<" NON REACHABLE POSITION "<<endl;
                    continue;
                }

                move_to(pos,turn_rot,rate,true);
                open_gripper(); 

            }else if(rot[1] != 0){ // di lato , due rotazioni

                cout << "TURNED LEGO (X as height)" <<endl;

                int altezza = int(lego.model[1]) -48; // se è sdraiato è la x che da la sua altezza
                float altezza_cm = (altezza - 1 ) * UNIT_BLOCCHETTO;

                pos(2) = pos(2) - altezza_cm + 0.005; // 0.005 un offset sennò andava troppo basso

                rot << -rot[2],0,0;

                if(check_point(pos,rot)){
                    cout <<" REACHABLE POSITION" <<endl;
                }else{
                    cout <<" NON REACHABLE POSITION "<<endl;
                    cout << pos << endl;
                    continue;
                }

                open_gripper();
                move_to(pos,rot,rate,false);
                close_gripper();
                turn_rot << M_PI/2,0,-M_PI/2;
                pos(2) = 0.82;
                pos << -0.1 , -0.3 , 0.82;

                if(check_point(pos,turn_rot)){
                    cout <<" REACHABLE POSITION" <<endl;
                }else{
                    cout <<" NON REACHABLE POSITION "<<endl;
                    continue;
                }

                move_to(pos,turn_rot,rate,true);
                open_gripper();

                //turn the second time
                rot << M_PI/2,0,0;
                move_to(pos,rot,rate,false);
                close_gripper();
                rot << M_PI/2,0,-M_PI/2;

                if(check_point(pos,rot)){
                    cout <<" REACHABLE POSITION" <<endl;
                }else{
                    cout <<" NON REACHABLE POSITION "<<endl;
                    continue;
                }
                move_to(pos,turn_rot,rate,true);
                open_gripper();
            }
        }

    }

}

/**
 * @brief Read from the topic the actual value of the gripper joint (used only for simulation)
 * 
 * @return GripperState 
 */
GripperState return_gripper_states(){

    ros::NodeHandle node_2;
    ros::Duration Timeout = ros::Duration(3);

    boost::shared_ptr<const sensor_msgs::JointState_<std::allocator<void>>> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states",node_2, Timeout );
    actual_gripper[0]= msg->position[1];
    actual_gripper[1]= msg->position[2];

    return actual_gripper;
}


/**
 * @brief check that the position, with e orientation, entered is reachable by the robot , to do so check that at least one resultant of the inverse kinematics put inside the  
 *        direct kinematics gives the same position that we have put inside the inverse kinematics
 *        Is important to notice that the results of this control have 2 parameters, beacuse also the orientation that we want to achive has a lot of 
 *        importance in this control 
 * 
 * @param _pos position that you want to know if is reachable or not
 * @param e Orientation that you want to have in the position _pos
 * @return true 
 * @return false 
 */
bool check_point(PositionVector _pos,EulerVector e ){



    vector<JointStateVector> inverseSolution = inverse_kinematics(_pos,eul2rot(e));
    for(JointStateVector res : inverseSolution){

            double th1 = res[0];
            double th2 = res[1];
            double th3 = res[2];
            double th4 = res[3];
            double th5 = res[4];
            double th6 = res[5];
            
            DirectResult res_d = direct_kinematics(th1,th2,th3,th4,th5,th6);
            PositionVector p ;
            p = res_d.pos;

            for(int i=0;i<3;i++){

                if (!(float(res_d.pos[i]) - float(_pos[i])) < 0.001){

                    break;        
                }

                return true;
            }
    }
    return false;

}
/**
 * @brief Open the gripper of the robot, in the real robot it use the service call to move_gripper, in the simulation we public directly on the topic
 *        the angles that we want to reach with the fingers of the gripper
 * 
 * @return ** void 
 */
void open_gripper(){

   if(real_robot){
        ros::NodeHandle node_gripper;
        ros::ServiceClient client_gripper = node_gripper.serviceClient<ros_impedance_controller::generic_float>("move_gripper"); 
        ros_impedance_controller::generic_float::Request req;
        ros_impedance_controller::generic_float::Request resp;
        req.data = 70;
        cout << "req : "<< req << endl;
        bool res = client_gripper.call(req,resp);
        cout << "res :" << res << endl;

    }else{

        if(soft_gripper){
            JointStateGripperVector msg ;
            JointStateVector actual_pos = return_joint_states();
            ros::Rate loop_rate(loop_frequency);

            actual_gripper = return_gripper_states();
            while(actual_gripper(0)< 0.3){
                for(int i=0;i<6;i++){
                    msg(i)= actual_pos(i);
                }
                for(int i=6;i<8;i++){
                    msg(i)= actual_gripper(i-6);
                }
                send_des_jstate(msg);
                actual_gripper(0)=actual_gripper(0)+0.1;
                actual_gripper(1)=actual_gripper(1)+0.1;

                loop_rate.sleep();
        }
    
        }
    }

}

/**
 * @brief Close the gripper of the robot, in the real robot it use the service call to move_gripper, in the simulation we public directly on the topic
 *        the angles that we want to reach with the fingers of the gripper
 * 
 * @return ** void 
 */

void close_gripper(){
    
    if(real_robot){
        ros::NodeHandle node_gripper;
        ros::ServiceClient client_gripper = node_gripper.serviceClient<ros_impedance_controller::generic_float>("move_gripper"); 
        ros_impedance_controller::generic_float::Request req;
        ros_impedance_controller::generic_float::Request resp;
        req.data = 20;
        cout << "req : "<< req << endl;
        bool res = client_gripper.call(req,resp);
        cout << "res :" << res << endl;

    }else{
        if(soft_gripper){
            JointStateGripperVector msg ;
            JointStateVector actual_pos = return_joint_states();
            ros::Rate loop_rate(loop_frequency);

            actual_gripper = return_gripper_states();
            while(actual_gripper(0)> -0.3){
                for(int i=0;i<6;i++){
                msg(i)= actual_pos(i);
                }
                for(int i=6;i<8;i++){
                msg(i)= actual_gripper(i-6);
                }
                send_des_jstate(msg);
                actual_gripper(0)=actual_gripper(0)-0.1;
                actual_gripper(1)=actual_gripper(1)-0.1;

                loop_rate.sleep();
            }
        }

    }

}

int main(int argc,char **argv){



    ros::init(argc, argv, "custom_joint_publisher_main");
    ros::NodeHandle node;
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(loop_frequency);

    JointStateGripperVector amp;
    JointStateGripperVector freq;
    PositionVector pos_des;
    if(real_robot){
        jointState_msg_robot.data.resize(6);
    }else{
        if(soft_gripper){
            jointState_msg_robot.data.resize(8);
        }else{
            jointState_msg_robot.data.resize(9);

        }

    }

    float x,y,z;
    while (ros::ok())
    {   
        listen_lego_detection_turn(loop_rate);


        // cout << " x " ;
        // cin >> x;
        // cout << " y " ;
        // cin >> y;
        // y=-y;
        // cout << " z " ; // -0.87 nel reale 
        // cin >> z;
        // z=-z;
        // pos_des << x,y,z;    
        // EulerVector e ;
        // e << M_PI/2,0,0; // default braccio drittto 


        // if(check_point(pos_des,e)){
        //     cout <<" REACHABLE POSITION" <<endl;
        // }else{
        //     cout <<" NON REACHABLE POSITION "<<endl;
        //     continue;
        // }
        // open_gripper();
        // move_to(pos_des,e,loop_rate,false);
        // close_gripper();

        // for turn
        // e << M_PI/2,0,-M_PI/2; // default braccio drittto 
        // pos_des(2) = 0.82;

        // if(check_point(pos_des,e)){
        //     cout <<" REACHABLE POSITION" <<endl;
        // }else{
        //     cout <<" NON REACHABLE POSITION "<<endl;
        //     continue;
        // }
        // turn(pos_des,e,loop_rate);
        // open_gripper();


        loop_rate.sleep();
    }
    
    return 0;
}



