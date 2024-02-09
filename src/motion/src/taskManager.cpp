// Task manager node
/*
 * TODO:
 * Listen on vision topic
 * When message arrives (position of the block) send the coordinates of the frame to the movement node
 * Wait till the ack of the movement node
 * When ack arrives send an ack to the vision node to send another message with another position
 * Continue listen while stop
 *
 */
// -------------- START HEADER -------------- //

// -------------- Includes --------------
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

// -------------- Constants --------------
#define RATE 1       // Refresh rate

// -------------- Namespaces --------------
using namespace std;         // outputs
using namespace ros;         // ros library
using namespace Eigen;       // eigen library

// -------------- Publishers ---------------
Publisher motionPublisher;   // Publisher for sending the brick position to the motion topic
Publisher visionAck;         // Publisher for sending ack to vision topic
// -------------- Subscribers --------------
Subscriber subVision;        // Subscriber for listen on vision topic
Subscriber subAck;           // Subscriber for listen on motion topic an ack and tell the vision that it can send another msg
Subscriber subStop;          // Subscriber for listen on taskManager topic when the motion have stopped

// -------------- Flags --------------
// conventions: 0 false | 1 true
int msgVision = 0;           // if msg from vision arrived then start do the task
int ready = 1;               // Motion has finished so it can do new operations
int stop = 0;                // Task manager have to stop

// -------------- Positions --------------
Vector3f blockPosition;      // Position of the block
Vector3f blockRotation;      // Rotation of the block

// -------------- Classes --------------
int blockClassId;

// -------------- Prototipes --------------
void visionCallback(const motion::pos::ConstPtr &msg);
void ackCallback(const std_msgs::Int32::ConstPtr &msg);
void stopCallback(const std_msgs::Int32::ConstPtr &msg);
void ack();
void avviso();

// -------------- END HEADER -------------- //

int main(int argc, char ** argv)
{
    // DEBUG:
    cout << "TASK MANAGER START" << endl;

    // initialization of ros parameters
    init(argc, argv, "taskManager");
    NodeHandle nh;

    // Initializations of publishers
    motionPublisher = nh.advertise<motion::pos>("/motion/pos", 1);
    visionAck = nh.advertise<std_msgs::Int32>("/vision/ack", 1);

    // Initializations of subscribers
    subVision = nh.subscribe("/vision/pos", 1, visionCallback);
    subAck = nh.subscribe("/motion/ack", 1, ackCallback);
    subStop = nh.subscribe("/taskManager/stop", 1, stopCallback);

    // Set the refresh rate
    Rate loopRate(RATE);

    avviso();

    while (ros::ok())
    {
        // DEBUG
        //cout << "Sono nell'ok, msgVision: "<<msgVision<<" ready: "<<ready<<" stop: "<<stop<< endl;
        /*while (motionPublisher.getNumSubscribers() < 1)
            loopRate.sleep();

        // DEBUG
        /*
        cout << "Ho avuto un numero >= 1 di subscribers " << endl;
        */

        // If vision msg is received and the motion is finished, send the position to the motion
        
        // Manage the new arrive of messages
        spinOnce();
    }
    return 0;
}


/**
 * Set the block position, rotation and class received from the vision topic
 *
 * @param msg: message received
 */
void visionCallback(const motion::pos::ConstPtr &msg){
    // DEBUG
    //cout << "Msg received on /vision/pos!" << endl;

    // Get the data from the message
    blockPosition << msg->x, msg->y, msg->z;
    blockRotation << msg->roll, msg->pitch, msg->yaw;
    blockClassId = msg->class_id;
    
    msgVision = 1;

    motion::pos coord;

    if (msgVision && ready && !stop){

            ready = 0; // Prepare and start the movement

            //DEBUG
            cout<<"#####################################"<<endl;
            cout << "Blocks coordinates received from vision" << endl;
            cout << "Block position: " << blockPosition.transpose() << endl;
            cout << "Block rotation: " << blockRotation.transpose() << endl;
            cout << "Block class: " << blockClassId << endl;
            cout<<"#####################################"<<endl;


            // Get block position from world frame to base frame
            //blockPosition = worldToBase(blockPosition);

            // Set the frame coordinates for the movement topic
            coord.x = blockPosition(0);
            coord.y = blockPosition(1);
            coord.z = 0.86;

            // Set the rotations
            coord.roll = blockRotation(0);
            coord.pitch = blockRotation(1);
            coord.yaw = blockRotation(2);

            // Set the class id of the block
            coord.class_id = blockClassId;

            // Now that I have received the msg so the vision frame can send another message
            msgVision = 0;

            // DEBUG: Mostra quando il messaggio viene inviato al nodo di movimento
            cout << "Invio del messaggio al nodo di movimento "<<endl<<"******************"<<endl << coord << "******************" << endl;

            // Send the me<<endlssag
            // Se with the coordinates to the motion block
            motionPublisher.publish(coord);
        }


    // DEBUG: Mostra i dati del messaggio ricevuto
    //cout << "Posizione del blocco: " << blockPosition.transpose() << endl;
    //cout << "Rotazione del blocco: " << blockRotation.transpose() << endl;
    //cout << "ID della classe del blocco: " << blockClassId << endl;  
    
    
}

void avviso(){
    cout << "Sono nell'ok, msgVision: "<<msgVision<<" ready: "<<ready<<" stop: "<<stop<< endl;
}

/**
 * Tell when the motion has finished to operate, so I can do new operations
 *
 * @param msg: message received
 */
void ackCallback(const std_msgs::Int32::ConstPtr &msg)
{
    // DEBUG: Mostra quando arriva un ack dal nodo di movimento
    cout << "Ack ricevuto dal nodo di movimento" << endl;

    // Set data
    ready = msg->data;
    ack();
}

// Called when the motion node stopped to move, so I can send the ack to the vision node
void ack(){
    Rate loopRate(RATE);
    std_msgs::Int32 ack;
    ack.data = 1;

    // DEBUG: Mostra quando l'ack viene inviato al nodo di visione
    cout << "Invio dell'ack al nodo di visione" << endl;

    visionAck.publish(ack);
}

// If stop = 1 then stop the program have to stop
void stopCallback(const std_msgs::Int32::ConstPtr &msg){
    // DEBUG: Mostra quando arriva un messaggio di stop dal task manager
    cout << "Messaggio di stop ricevuto dal task manager" << endl;

    stop = msg->data;
}
