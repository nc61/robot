#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/IR.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "SenseReact.h"
#include "Common.h"

//IR information and states
uint8_t mode = WANDER;
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
char wait;

//Establish global scope for publisher
ros::Publisher motorPub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Wander");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    //Publisher
    motorPub = nh.advertise<robot::motor>("motor_command", 1000);
    loop_rate.sleep();
    //Subscriber
    ros::Subscriber IRSub = nh.subscribe<robot::IR>("sensor_data", 2, processIR);
    ros::Subscriber xmegaSub = nh.subscribe<std_msgs::UInt8>("xmega_feedback", 1, xmegaFeedback);

    while(ros::ok())
    {
        //In wander mode, robot moves forward with a duty cycle of 70
        if (mode == WANDER)
        {
            if ((midIR > MID_VN) || (leftIR > LEFT_VN) || (rightIR > RIGHT_VN))
            {
                avoid_obstacle();
            }

            sendCommand(GO_FORWARD, 75);
            ros::spinOnce();
        }
        loop_rate.sleep();
    }
}

//Description: avoids an obstacle
//Calls: stop(), sendCommand()
void avoid_obstacle()
{
    if (midIR > MID_VN)
    {
        ROS_WARN("Object in front");
        stop();
        sendCommand(GO_BACKWARD, 70);
        while(midIR > MID_FAR) { ros::spinOnce(); }
        stop();
        (leftIR > rightIR) ? sendCommand(PIVOT_RIGHT, 70) : sendCommand(PIVOT_LEFT, 70) ; 
        while (midIR > MID_FAR){ ros::spinOnce(); }
    } else if (leftIR > LEFT_VN) {
        stop();
        ROS_WARN("Object to left");
        sendCommand(PIVOT_RIGHT, 70);
        while (leftIR > LEFT_FAR) { ros::spinOnce(); }
    } else if (rightIR > RIGHT_VN) {
        stop();
        ROS_WARN("Object to right");
        sendCommand(PIVOT_LEFT, 70);
        while (rightIR > RIGHT_FAR) { ros::spinOnce(); }
    }
}

//Description: Tells motors to stop, then
//     waits for signal that motors have stopped
void stop()
{
    robot::motor msg;
    msg.command = STOP_MOTORS;
    msg.duty_cycle = 0;
    motorPub.publish(msg);
    
    wait = 0;
    while (wait != 1){ ros::spinOnce(); }
}

//Description: Sends a command to the motor controller
void sendCommand(uint8_t command, uint8_t duty_cycle)
{
    robot::motor msg;
    msg.command = command;
    msg.duty_cycle = duty_cycle;
    motorPub.publish(msg);
}

/**********************************************************
 * Callback functions
 **********************************************************/

//Description: updates global variables (sensor information)
//Called by: spinOnce()
void processIR(const robot::IR::ConstPtr &msg)
{
    leftIR = msg->leftIR;
    midIR = msg->midIR;
    rightIR = msg->rightIR;
}

//Description: Updates global variable (feedback from xmega)
//Called by: spinOnce();
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg)
{
    wait = msg->data;
    ROS_INFO("wait value: %d", wait);
}

