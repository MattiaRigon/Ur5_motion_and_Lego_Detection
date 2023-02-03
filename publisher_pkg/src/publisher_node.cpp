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
#include <cmath>



// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
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


using namespace std;


void send_des_jstate(const JointStateVector & joint_pos)
{


    for (int i = 0; i < joint_pos.size() -2; i++)
    {
        jointState_msg_robot.data[i] = joint_pos[i];
    }
    // for (int i=6;i<8;i++){
    //     jointState_msg_robot.data[i] = actual_gripper[i-6];
    // }
    pub_des_jstate.publish(jointState_msg_robot);



}


JointStateVecor return_joint_states(){

    ros::NodeHandle node_1;
    ros::Duration Timeout = ros::Duration(3);

    JointStateVecor actual_pos ;

    boost::shared_ptr<const sensor_msgs::JointState_<std::allocator<void>>> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states",node_1, Timeout );
    // actual_pos[0]= msg->position[4];
    // actual_pos[1]= msg->position[3];
    // actual_pos[2]= msg->position[0];
    // actual_pos[3]= msg->position[5];
    // actual_pos[4]= msg->position[6];
    // actual_pos[5]= msg->position[7];


    actual_pos[0]= msg->position[2];
    actual_pos[1]= msg->position[1];
    actual_pos[2]= msg->position[0];
    actual_pos[3]= msg->position[3];
    actual_pos[4]= msg->position[4];
    actual_pos[5]= msg->position[5];

    return actual_pos;
}
void move_to(PositionVecor pos,EulerVector e ,ros::Rate rate){

    //EulerVector e ;
    //e << M_PI/2,0,0; // default braccio drittto 

    if(pos[0] == 0){
        pos[0]= 0.001;
    }
    if(pos[1] == 0){
        pos[1]= 0.001;
    }

    vector<PositionVecor> intermediate;

    JointStateVecor actual_pos = return_joint_states();

    DirectResult direct_res = direct_kinematics(actual_pos(0),actual_pos(1),actual_pos(2),actual_pos(3),actual_pos(4),actual_pos(5));

    PositionVecor i1 ;
    i1 = direct_res.pos;
    i1(2)=0.5;
    intermediate.push_back(i1);

    PositionVecor i ;
    i<< pos[0],pos[1],0.5;
    intermediate.push_back(i);

    double dt = 0.001;
    double DtP = 2;
    double DtA = 0.5;
    JointStateVector pos_send;
    PositionVecor pos_check;
    pos_send<<0,0,0,0,0,0,0,0;

    ros::Rate loop_rate(loop_frequency);

    auto res = p2pMotionPlanIntermediatePoints(actual_pos,pos,e,intermediate,0.001);
    
    // fix se va a addosso a qualcosa deve andare a richiamare p2pMotionPlan però cambaindo configurazione finale
    // cambaire modo di fare p2pMotionPlan , far si che calcola il prossimo q da raggiungere , prova ad andarci , guarda se si scontrerebbe addosso a qualcosa
    // se si sceglie il secondo nearest, e cosi via finchè trova una configurazione che gli permette di non scontrarsi contro nulla

    for (vector<double> conf : res){
        if(!check_singularity_collision(conf[1],conf[2],conf[3],conf[4],conf[5],conf[6])){
            cout << "COLLISIONE CON DELLE SINGOLARITA' "<<endl;
            cout <<"Per andare da : "<<direct_res.pos<<endl;
            cout << "a : "<< pos <<endl;

            break;
        }
    }

    for (vector<double> conf : res){

        for(int i=1;i<7;i++){

            pos_send(i-1)=conf[i];

        }
        //cout << pos_send << endl;
        send_des_jstate(pos_send);
        loop_rate.sleep();

    }

}

void listen_lego_detection(ros::Rate rate){

    ros::NodeHandle node_1;
    //ros::Duration Timeout = ros::Duration(1);
    //Json::Value msg_recived;

    spawnLego_pkg::legoGroup::ConstPtr msg = ros::topic::waitForMessage<spawnLego_pkg::legoGroup>("/lego_position",node_1 );
    if(msg!=0){
        cout << "Arrivato messaggio" << endl;
        vector<spawnLego_pkg::legoDetection> vector = msg->lego_vector ;
        for (spawnLego_pkg::legoDetection lego : vector){

            cout << lego.model << endl;
            PositionVecor pos;

            pos << lego.pose.position.x-0.5,-(lego.pose.position.y-0.35),-(lego.pose.position.z-1.75+0.14);
            Quaternion q ;
            q.x = lego.pose.orientation.x;
            q.y = lego.pose.orientation.y;
            q.z = lego.pose.orientation.z;
            q.w = lego.pose.orientation.w;

            EulerVector rot  = ToEulerAngles(q); 
            rot << -rot[2],0,0;
            //cout << pos << endl;
            //cout << rot << endl;
            if(check_point(pos)){
                cout <<" POSIZIONE RAGGIUNGIBILE " <<endl;
            }else{
                cout <<" POSIZIONE NON RAGGIUNGIBILE "<<endl;
                cout << pos << endl;
                continue;
            }
            move_to(pos,rot,rate);
            cout << endl ;
            cout << endl ;
            cout << endl ;

            
        }


    }else{
        cout << "vuoto " << endl;
    }


}




GripperState return_gripper_states(){

    ros::NodeHandle node_2;
    ros::Duration Timeout = ros::Duration(3);

    boost::shared_ptr<const sensor_msgs::JointState_<std::allocator<void>>> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states",node_2, Timeout );
    actual_gripper[0]= msg->position[1];
    actual_gripper[1]= msg->position[2];

    return actual_gripper;
}



bool check_point(PositionVecor _pos){


    // chiede gli angoli alla inverse kinematics che tra la lista di 8 array guarda che ci sia almeno un array di angoli che 
    // rispetta le condizioni, ovvero angoli la cui applicazione non comportano che nessun joint sbatta sul soffitto , ovvero abbia z > 0
    // e che gli angoli che bisogna applicare si possano veramente 

    EulerVector e(0,0,0);
    vector<JointStateVecor> inverseSolution = inverse_kinematics(_pos,eul2rot(e));
    for(JointStateVecor res : inverseSolution){
            double th1 = res[0];
            double th2 = res[1];
            double th3 = res[2];
            double th4 = res[3];
            double th5 = res[4];
            double th6 = res[5];

            //controlla che la posizione inserita sia raggiungibile dal robot , per farlo controlla che il risultaro della inverse messo dentro la direct dia la medesima posizione 
            
            DirectResult res_d = direct_kinematics(th1,th2,th3,th4,th5,th6);
            PositionVecor p ;
            p = res_d.pos;

            for(int i=0;i<3;i++){

                if ((float(res_d.pos[i]) - float(_pos[i])) < 0.001){
                    continue;
                }else{
                    //cout << float(res_d.pos[i]) << " " << float(_pos[i]) << endl;
                    return false;        
                }
            }
    }
    return true;


}

void open_gripper(){
    
    JointStateVector msg ;
    JointStateVecor actual_pos = return_joint_states();
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
        actual_gripper(0)=actual_gripper(0)+0.01;
        actual_gripper(1)=actual_gripper(1)+0.01;

        loop_rate.sleep();
    }
}

void close_gripper(){
    

    JointStateVector msg ;
    JointStateVecor actual_pos = return_joint_states();
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
        actual_gripper(0)=actual_gripper(0)-0.01;
        actual_gripper(1)=actual_gripper(1)-0.01;

        loop_rate.sleep();
    }
}

int main(int argc,char **argv){



    ros::init(argc, argv, "custom_joint_publisher_main");
    ros::NodeHandle node;
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(loop_frequency);

    JointStateVector amp;
    JointStateVector freq;
    PositionVecor pos_des;
    jointState_msg_robot.data.resize(6);

    float x,y,z;
    while (ros::ok())
    {   
        //listen_lego_detection(loop_rate);
        cout << " x " ;
        cin >> x;
        cout << " y " ;
        cin >> y;
        y=-y;
        cout << " z " ;
        cin >> z;
        z=-z;
        pos_des << x,y,z;    
        EulerVector e ;
        e << M_PI/2,0,0; // default braccio drittto 


        if(check_point(pos_des)){
            cout <<" POSIZIONE RAGGIUNGIBILE " <<endl;
        }else{
            cout <<" POSIZIONE NON RAGGIUNGIBILE "<<endl;
            continue;
        }
        //open_gripper();
        move_to(pos_des,e,loop_rate);
        //close_gripper();

        loop_rate.sleep();
    }
    
    return 0;
}
