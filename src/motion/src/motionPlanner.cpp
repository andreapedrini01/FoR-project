/**
 * @file motionPlanner.cpp
 * @author Pieropan Lorenzo (lorenzo.pieropan@unitn.it)
 * @brief  This file contains the motion planner node
 * @version 1
 * @date 2023-12-29
 */
 
#include "motion/kinematics.h"
#include "motion/pos.h"
#include "motion/generic_float.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <complex>

// ----------- DEFINES ----------- //
/// Loop rate of the node
#define LOOP_RATE 1000
/// Number of joints
#define JOINTS 8

// ----------- NAMESPACES ----------- //

using namespace std;
using namespace Eigen;

// ----------- STRUCTS ----------- //

/**
 * @brief This struct contains the position and orientation of the end-effector
 */
struct Pose
{
    Vector3f position;
    Vector3f orientation;
};

// ----------- GLOBAL VARIABLES ----------- //
/// Position of the end-effector
Pose pose;
/// Class of the block
int class_id;
/// Flag to check if it have to grasp
int grasp = 0;
/// Publisher for the desired joint state
ros::Publisher pub_des_jstate;
/// Publisher for the ack
ros::Publisher ack_pos;
/// Subscriber for the position msg
ros::Subscriber sub_pos;
/// Flag to check if it is in simulation
int real_robot = 0;
/// @brief Initial joint configuration
VectorXf TH0(6);
/// @brief Flag to check if it is the first time that the node is called
int first = 1;
/// @brief Max time for the trajectory
double maxT = 10;


// ----------- FUNCTION PROTOTIPES ----------- //
VectorXf invDiffKinematicControlCompleteQuat(VectorXf q, Vector3f xe, Vector3f xd, Vector3f vd, Quaternionf qe, Quaternionf qd, Vector3f wd);
Vector3f pd(double t, Vector3f xef, Vector3f xe0);
Quaternionf qd(double tb, Quaternionf qf, Quaternionf q0);
Quaternionf quatMult(Quaternionf q1, Quaternionf q2);
void invDiffKinematicControlSimCompleteQuat(Vector3f xef, Vector3f phief, float dt);
void posCallback(const motion::pos::ConstPtr &msg); //receives e-e position request
void sendJointState(VectorXf q);
void move();
void startingPosition();
void ack();

// ----------- MAIN ----------- //

int main(int argc, char **argv)
{
    // initialize ros, node handler and publisher
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;
    ROS_INFO("MOTION PLANNER HAS STARTED");

    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ack_pos = node.advertise<std_msgs::Int32>("/motion/ack", 1);

    sub_pos = node.subscribe("/motion/pos", 1, posCallback);

    startingPosition();

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

// ----------- FUNCTION DEFINITIONS ----------- //

/**
 * @brief           This function is used to calculate the joint config matrix using the inverse differential kinematics
 *
 * @param xef       desired end-effector position
 * @param phief     desired end-effector orientation
 * @param dt        time step
 * 
 * @return      VectorXf joint config vector
 */
void invDiffKinematicControlSimCompleteQuat(Vector3f xef, Vector3f phief, float dt)
{
    frame now;   // current frame
    frame start; // start frame
    if  (first)
        TH0 << -0.31685, -0.77687, -2.09686, -1.63, -1.57, 3.49; // initial joint configuration
    first = 0;
    start = directKin(TH0);

    VectorXf qk = TH0;           // set the initial joint config
    VectorXf qk1(6);             // joint config homing

    Matrix3f kp; // position gain
    kp = Matrix3f::Identity() * 10;

    Matrix3f kq; // orientation gain
    kq = Matrix3f::Identity() * (-10);

    VectorXf qdotk(6); // joint velocities coefficients

    Vector3f vd; // desired linear velocity

    Matrix3f Re, R0, Rd;
    Quaternionf qe, q0, qf;
    Quaternionf work;

    Vector3f wd;

    R0 = start.rot;
    q0 = R0;

    Rd = eul2rotm(phief);
    qf = Rd;

    // loop
    for (float i = dt; i < maxT; i += dt)
    {
        now = directKin(qk); // get the current frame

        Re = now.rot;
        qe = Re;

        vd = (pd(i, xef, start.xyz) - pd(i - dt, xef, start.xyz)) / dt; // desired linear velocity

        work = quatMult(qd(i+dt, qf, q0), (qd(i, qf, q0).conjugate()));
        wd = work.vec()*(2/dt);

        // coefficient
        qdotk = invDiffKinematicControlCompleteQuat(qk, now.xyz, pd(i, xef, start.xyz), vd, qe, qd(i, qf, q0), wd);

        // euler integration
        qk1 = qk + qdotk * dt;
        qk = qk1;
        sendJointState(qk);
    }
    TH0 = qk;
    cout<<"## movimento completato ##"<<endl;
}

/**
 * @brief      This function is used to calculate the joint velocities using the jacobian matrix
 *
 * @param[in]  q     The current joint config
 * @param[in]  xe    The current end-effector position
 * @param[in]  xd    The desired end-effector position
 * @param[in]  vd    The desired end-effector linear velocity
 * @param[in]  qe    The current end-effector orientation
 * @param[in]  qd    The desired end-effector orientation
 * @param[in]  wd    The desired end-effector angular velocity
 *
 * @return     The joint velocities
 */
VectorXf invDiffKinematicControlCompleteQuat(VectorXf q, Vector3f xe, Vector3f xd, Vector3f vd, Quaternionf qe, Quaternionf qd, Vector3f wd)
{
    MatrixXf J;
    J = jacobian(q);

    Quaternionf qeo = quatMult(qd, (qe.conjugate()));
    Vector3f eo = qeo.vec();

    VectorXf qdot;
    Matrix3f kp = Matrix3f::Identity()*10;
    Matrix3f kq = Matrix3f::Identity()*10;
    

    if (J.determinant()<0.001){
      ROS_INFO("Near singular configuration");
    }
    
    Vector3f Mkp, Mko;
    Mkp = vd + kp*(xd-xe);
    Mko = wd + kq*eo;

    Matrix<float, 6, 1> Mk;
    Mk.block<3, 1>(0, 0) = Mkp;
    Mk.block<3, 1>(3, 0) = Mko;
    qdot = J.inverse() * Mk;

    return qdot;
}

/**
 * @brief      This function is used to calculate trajectory for the end-effector position
 *
 * @param[in]  t     The current time
 * @param[in]  xef    The desired end-effector position
 * @param[in]  xe0    The start end-effector position
 *
 * @return     The end-effector position
 */
Vector3f pd(double t, Vector3f xef, Vector3f xe0)
{
    double t_norm = t / maxT;
    if (t_norm > 1)
    {
        return xef;
    }
    else
    {
        return t_norm * xef + (1 - t_norm) * xe0;
    }
}

/**
 * @brief      This function is used to calculate trajectory for the end-effector orientation
 *
 * @param[in]  t     The current time
 * @param[in]  xef    The desired end-effector position
 * @param[in]  xe0    The start end-effector position
 *
 * @return     The end-effector position
 */
Quaternionf qd(double tb, Quaternionf qf, Quaternionf q0){  

    double t = tb/maxT;

    if(t>1)
        return qf;
    else
        return q0.slerp(t, qf);
    
    
}


/**
 * @brief quaternion product
 */
Quaternionf quatMult(Quaternionf q1, Quaternionf q2){
    Quaternionf resultQ;
    resultQ.setIdentity();

    resultQ.w() = q1.w() * q2.w() - q1.vec().dot(q2.vec());
    resultQ.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());

    return resultQ;
}


/**
 * @brief CALLBACK function for the position topic
 * 
 * @param msg message received
 */
void posCallback(const motion::pos::ConstPtr &msg)
{
    pose.position(0) = msg->x-0.5;
    pose.position(1) = -(msg->y)+0.35;
    pose.position(2) = msg->z;

    pose.orientation(2) = msg->roll;
    pose.orientation(1) = msg->pitch;
    pose.orientation(0) = -(msg->yaw);

    class_id = msg->class_id;

    move();
}

/**
 * @brief      This function is used to send the joint states to the robot
 *
 * @param  q     The joint config
 */
void sendJointState(VectorXf q)
{
    ros::Rate loop_rate(LOOP_RATE);
    std_msgs::Float64MultiArray jointState_msg_robot;
    jointState_msg_robot.data.resize(JOINTS);
    for (int i = 0; i < 6; i++)
    {
        jointState_msg_robot.data[i] = q(i);
    }
    if (!real_robot)
    {
        if (grasp)
        {          
            jointState_msg_robot.data[6] = -0.20;
            jointState_msg_robot.data[7] = -0.20;
        }
        else
        {
            jointState_msg_robot.data[6] = 1.0;
            jointState_msg_robot.data[7] = 1.0;
        }
    }

    pub_des_jstate.publish(jointState_msg_robot);
    loop_rate.sleep();
}

/**
 * @brief     This function is used to move the robot according to the block position
 */
void move()
{
    ros::Rate loop_rate(LOOP_RATE);
    float dt = 0.01; // time step
    Vector3f target;
    
    // go above the desired position
    target << pose.position(0), pose.position(1), 0.76;
    invDiffKinematicControlSimCompleteQuat(target, pose.orientation, dt);
    
    // go to the desired position
    target << pose.position(0), pose.position(1), pose.position(2);
    invDiffKinematicControlSimCompleteQuat(pose.position, pose.orientation, dt);
    
    // grasp
    grasp = 1;
    sendJointState(TH0);
    

    for (int i = 0; i < 20; i++)
    {
        loop_rate.sleep();
    }

    // lift a little bit
    target(2) = 0.76;
    invDiffKinematicControlSimCompleteQuat(target, pose.orientation, dt);

    startingPosition();

    // go to the desired position depending on the class of the block
    switch (class_id)
        {
        case 0:
            target << 0.4, 0.0, 0.82;
            break;

        case 1:
            target << 0.25, -0.26, 0.82;
            break;

        case 2:
            target << 0.35, -0.26, 0.82;
            break;

        case 3:
            target << 0.25, 0.0, 0.82;
            break;

        case 4:
            target << 0.35, 0.0, 0.82;
            break;

        default:
            break;
        }
    invDiffKinematicControlSimCompleteQuat(target, pose.orientation, dt);
    
    // ungrasp
    grasp = 0;
    sendJointState(TH0);
    
    for (int i = 0; i < 20; i++)
    {
        loop_rate.sleep();
    }

    // lift a little bit
    target(2) = 0.76;
    invDiffKinematicControlSimCompleteQuat(target, pose.orientation, dt);
    
    // go to the starting position
    startingPosition();

    cout<<"##### OPERAZIONE COMPLETATA #####"<<endl;
}

/**
 * @brief      This function is used to go to the starting position
 */
void startingPosition()
{
    float dt; // time step
    dt = 0.01;
    Vector3f targetp, targeto;
    targetp << 0.0, -0.4, 0.7;
    targeto << 0.0, 0.0, 0.0;
    invDiffKinematicControlSimCompleteQuat(targetp, targeto, dt);
    ack();
}

/**
 * @brief     This function is used to send the ack to the taskManager
 *              - send it when the robot has finished the motion task
 */
void ack()
{
    ros::Rate loop_rate(LOOP_RATE);
    std_msgs::Int32 ack;
    ack.data = 1;
    // wait a little bit before sending the ack to the taskManager (not stress too much the robot)
    for (int i = 0; i < 40; i++)
    {
        loop_rate.sleep();
    }
    ack_pos.publish(ack);
    //ack.data = 0;
}
