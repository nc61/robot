#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/sensors.h"
#include "robot/color.h"
#include "robot/object.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "SenseReact.h"
#include "Demo.h"
#include "Common.h"

//IR information and states
uint8_t state = FIND_BIN;
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
uint8_t FSR;
uint8_t xmega_feedback;
char wait;
float xpos_pile, area_pile;
float xpos_bin, area_bin, y_0, y_1, y_2, y_3, y_avg_bin;

//Establish global scope for publisher
ros::Publisher motorPub;
ros::Publisher servoPub;
ros::Publisher statePub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Wander");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    //Publishers
    motorPub = nh.advertise<robot::motor>("motor_command", 1000);
    servoPub = nh.advertise<std_msgs::UInt8>("maestro_command", 1);
    statePub = nh.advertise<std_msgs::UInt8>("state_data", 1);
    loop_rate.sleep();
    //Subscribers
    ros::Subscriber sensorSub = nh.subscribe<robot::sensors>("sensor_data", 2, processSensors);
    ros::Subscriber xmegaSub = nh.subscribe<std_msgs::UInt8>("xmega_feedback", 1, xmegaFeedback);
    ros::Subscriber colorSub = nh.subscribe<robot::color>("color_data", 1, processColor);
    ros::Subscriber objectRecSub = nh.subscribe<robot::object>("object_data", 1, processObject);

    while(ros::ok())
    {
        std_msgs::UInt8 msg;
        msg.data = state;
        statePub.publish(msg);

        if (state == FIND_PILE)
        {
            ROS_INFO("Finding pile");
            sendMotorCommand(PIVOT_LEFT, 70);
            while ( (abs(xpos_pile - PILE_CENTERED) > 10) || (area_pile < AREA_PILE_THRESH) )
                { ros::spinOnce(); loop_rate.sleep(); }
            sendMotorCommand(GO_FORWARD, 70);
            state = NAV_TO_PILE;
        } else if (state == NAV_TO_PILE) {
            ROS_INFO("Navigating to pile");
            while (area_pile < AREA_PILE_CLOSE) { ros::spinOnce(); loop_rate.sleep(); }
            stop();
            state = APPROACH_PILE;
        } else if (state == APPROACH_PILE) {
            ROS_INFO("Approaching pile");
            sendServoCommand(DIG_BUCKET);
            sendMotorCommand(GO_FORWARD, 70);
            while (FSR_THRESHOLD < FSR_THRESHOLD){ ros::spinOnce(); loop_rate.sleep(); }
            sendServoCommand(LIFT_DIRT);
            stop();
            delayms(1000);
            sendMotorCommand(GO_BACKWARD, 70);
            delayms(1000);
            stop();
            state = FIND_BIN;
        } else if (state == FIND_BIN) {
            sendServoCommand(LIFT_DIRT);
            ROS_INFO("Finding bin");
            findBin(0);
            state = NAV_TO_BIN;
        } else if (state == NAV_TO_BIN) {
            ROS_INFO("Navigating to bin");
            navToBin();
            state = FIND_PILE;
        }

        loop_rate.sleep();
    }
}
void navToBin()
{
    ros::Rate loop_rate(LOOP_RATE);
    while (y_avg_bin < 125) 
    {
        sendMotorCommand(GO_FORWARD_IMM, 70);
        delayms(500);
        sendMotorCommand(STOP_IMMEDIATELY, 0);
        for (int i = 0; i < 5; i++) 
        {
            ros::spinOnce();
            ROS_INFO("y_avg: %f", y_avg_bin); 
            if (y_avg_bin >= 125)
                break;
            loop_rate.sleep();
        }
    }
    stop();
    if (y_3 - y_2 > 10)
    {
        ROS_INFO("y3 > y2");
        while (xpos_bin > 40) { 
            sendMotorCommand(PIVOT_RIGHT_IMM, 75);
            delayms(50);
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            for (int i = 0; i < 5; i++)
            {
                ros::spinOnce();
                ROS_INFO("xpos_bin: %f", xpos_bin);
                if (xpos_bin <= 40)
                    break;
                loop_rate.sleep();
            }
        }
        sendMotorCommand(GO_FORWARD, 75);
        while (leftIR < 275) { ros::spinOnce(); }
        sendMotorCommand(STOP_IMMEDIATELY, 0);
        sendMotorCommand(PIVOT_LEFT, 75);
        while (midIR < 315) { ros::spinOnce(); }
        stop();
    } else if (y_2 - y_3 > 10) {
        ROS_INFO("y3 > y2");
        while (xpos_bin < 260) { 
            sendMotorCommand(PIVOT_LEFT_IMM, 75);
            delayms(50);
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            for (int i = 0; i < 5; i++)
            {
                ros::spinOnce();
                ROS_INFO("xpos_bin: %f", xpos_bin);
                if (xpos_bin >= 260)
                    break;
                loop_rate.sleep();
            }
        }
        sendMotorCommand(GO_FORWARD, 75);
        while (rightIR < 275) { ros::spinOnce(); }
        sendMotorCommand(STOP_IMMEDIATELY, 0);
        sendMotorCommand(PIVOT_RIGHT, 75);
        while (midIR < 315) { ros::spinOnce(); }
        stop();
    }
}

//Description: Pivots until the bin is found then stops
//Calls: delayms(), sendMotorCommand();
void findBin(uint8_t dir)
{
    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok())
    {
        sendMotorCommand(PIVOT_LEFT_IMM, 0);
        delayms(400);
        sendMotorCommand(STOP_IMMEDIATELY, 0);
        for (int i = 0; i < 10; i++)
        {
            ros::spinOnce();
            ROS_INFO("area_bin: %f", area_bin);
            if ((area_bin > AREA_BIN_THRESH) && (xpos_bin > 125))
            {
                stop();
                return;
            }
            loop_rate.sleep();
        }
    }
    return;
}
//Description: avoids an obstacle
//Calls: stop(), SendMotorCommand()
void avoid_obstacle()
{
    if (midIR > MID_VN)
    {
        ROS_WARN("Object in front");
        sendMotorCommand(GO_BACKWARD, 70);
        while(midIR > MID_FAR) { ros::spinOnce(); }
        (leftIR > rightIR) ? sendMotorCommand(PIVOT_RIGHT, 70) : sendMotorCommand(PIVOT_LEFT, 70) ; 
        while (midIR > MID_VF){ ros::spinOnce(); }
    } else if (leftIR > LEFT_VN) {
        ROS_WARN("Object to left");
        sendMotorCommand(PIVOT_RIGHT, 70);
        while (leftIR > LEFT_FAR) { ros::spinOnce(); }
    } else if (rightIR > RIGHT_VN) {
        ROS_WARN("Object to right");
        sendMotorCommand(PIVOT_LEFT, 70);
        while (rightIR > RIGHT_FAR) { ros::spinOnce(); }
        stop();
    }
    if (state == NAV_TO_PILE)
        state = FIND_PILE;
    else if (state == NAV_TO_BIN)
        state = FIND_BIN;
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
void sendMotorCommand(uint8_t command, uint8_t duty_cycle)
{
    robot::motor msg;
    msg.command = command;
    msg.duty_cycle = duty_cycle;
    motorPub.publish(msg);
}

//Description: Sends a command to the mini Maestro servo controller
void sendServoCommand(uint8_t command)
{
    std_msgs::UInt8 msg;
    msg.data = command;
    servoPub.publish(msg);
}

void delayms(uint16_t ms)
{
    ros::Rate millisecond(1000);
    for (int i = 0; i < ms; i++) {millisecond.sleep(); }
}

/**********************************************************
 * Callback functions
 **********************************************************/

//Description: updates global variables (sensor information)
//Called by: spinOnce()
void processSensors(const robot::sensors::ConstPtr &msg)
{
    leftIR = msg->leftIR;
    midIR = msg->midIR;
    rightIR = msg->rightIR;
    FSR = msg->FSR;
}

//Description: Updates global variable (feedback from xmega)
//Called by: spinOnce()
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg)
{
    xmega_feedback = msg->data;
}

//Description: Updates global variable (x position of pile)
//Called by: SpinOnce()
void processColor(const robot::color::ConstPtr &msg)
{
    xpos_pile = msg->xpos;
    area_pile = msg->area;
}

//Description: updates global variables (xpos_bin and area_bin)
//Called by: spinOnce()
void processObject(const robot::object::ConstPtr &msg)
{
    y_0 = msg->y0;
    y_1 = msg->y1;
    y_2 = msg->y2;
    y_3 = msg->y3;
    y_avg_bin = (y_2 + y_3)/2;

    xpos_bin = msg->xpos;
    area_bin = msg->area;
}
