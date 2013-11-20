#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/sensors.h"
#include "robot/color.h"
#include "robot/tracking.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "SenseReact.h"
#include "Common.h"

//IR information and states
uint8_t state = FIND_PILE;
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
char wait;
float xpos_pile, area_pile;
float xpos_bin, area_bin;

//Establish global scope for publisher
ros::Publisher motorPub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Wander");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    //Publisher
    motorPub = nh.advertise<robot::motor>("motor_command", 1000);
    servoPub = nh.advertise<std_msgs::UInt8>("maestro_command", 1);
    loop_rate.sleep();
    //Subscriber
    ros::Subscriber sensorSub = nh.subscribe<robot::sensors>("sensor_data", 2, processSensors);
    ros::Subscriber xmegaSub = nh.subscribe<std_msgs::UInt8>("xmega_feedback", 1, xmegaFeedback);
    ros::Subscriber colorSub = nh.subscribe<robot::color>("color_data", 1, processColor);
    ros::Subscriber objectRecSub = nh.subscribe<robot::color>("object_data", 1, procesObject);

    while(ros::ok())
    {
        if (state == FIND_PILE)
        {
            ROS_INFO("Finding pile");
            //Pivot right until color is detected, then move forward
            sendMotorCmd(PIVOT_LEFT, 70);
            while ( (abs(xpos_pile - PILE_CENTERED) > PILE_XPOS_THRESH) || (area_pile < PILE_AREA_THRESH) ) { ros::spinOnce(); }
            sendMotorCmd(GO_FORWARD, 70);
            state = NAV_TO_PILE;
        } else if (state == NAV_TO_PILE) {
            ROS_INFO("Navigating to pile");
            while (area_pile < PILE_AREA_CLOSE) { ros::spinOnce(); }
            stop();
            state = APPROACH_PILE;
        } else if (state == APPROACH_PILE) {
            ROS_INFO("Approaching pile");
            sendServoCmd(DIG_BUCKET);
            sendMotorCmd(GO_FORWARD, 70);
            while (FSR_weight < FSR_THRESHOLD){ ros::spinOnce(); }
            sendMotorCmd(LIFT_DIRT);
            stop();
	        for(int i = 0; i < 20; i++){ loop_rate.sleep(); }
            sendMotorCmd(GO_BACKWARD, 70);
	        for(int i = 0; i < 10; i++){ loop_rate.sleep(); }
            stop();
            state = FIND_BIN;
        } else if (state == FIND_BIN) {
            ROS_INFO("Finding bin");
            
            sendMotorCmd(PIVOT_LEFT, 70);
	        for(int i = 0; i < 10; i++)
            {
                if ((area_bin > AREA_BIN_THRESH) && (xpos_bin > 125)) 
                {
                    sendMotorCmd(IMMEDIATE_STOP);
                    state = NAV_TO_BIN; 
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep(); 
            }

	        sendMotorCmd(IMMEDIATE_STOP);
            for(int i = 0; i < 10; i++)
            {
                if ((area_bin > AREA_BIN_THRESH) && (xpos_bin > 125)) 
                {
                    sendMotorCmd(IMMEDIATE_STOP);
                    state = NAV_TO_BIN; 
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep(); 
            }


        } else if (state == NAV_TO_BIN) {
            ROS_INFO("Navigating to bin");

            sendMotorCmd(GO_FORWARD, 70);
            for(int i = 0; i < 10; i++)
            {
                if ((area_bin > AREA_BIN_THRESH) && (xpos_bin > 125)) 
                {
                    sendMotorCmd(IMMEDIATE_STOP);
                    state = NAV_TO_BIN; 
                    break;
                }
                ros::spinOnce();
                loop_rate.sleep(); 
            }
        }

        loop_rate.sleep();
    }
}

//Description: avoids an obstacle
//Calls: stop(), sendMotorCmd()
void avoid_obstacle()
{
    if (midIR > MID_VN)
    {
        ROS_WARN("Object in front");
        sendMotorCmd(GO_BACKWARD, 70);
        while(midIR > MID_FAR) { ros::spinOnce(); }
        (leftIR > rightIR) ? sendMotorCmd(PIVOT_RIGHT, 70) : sendMotorCmd(PIVOT_LEFT, 70) ; 
        while (midIR > MID_FAR){ ros::spinOnce(); }
    } else if (leftIR > LEFT_VN) {
        ROS_WARN("Object to left");
        sendMotorCmd(PIVOT_RIGHT, 70);
        while (leftIR > LEFT_FAR) { ros::spinOnce(); }
    } else if (rightIR > RIGHT_VN) {
        ROS_WARN("Object to right");
        sendMotorCmd(PIVOT_LEFT, 70);
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
void sendMotorCmd(uint8_t command, uint8_t duty_cycle)
{
    robot::motor msg;
    msg.command = command;
    msg.duty_cycle = duty_cycle;
    motorPub.publish(msg);
}

//Description: Sends a command to the mini Maestro servo controller
void sendServoCmd(uint8_t command)
{
    std_msgs::UInt8 msg;
    msg.data = command;
    servoPub.publish(msg);
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
    //A 'reflex' response to the bump switch
    if ( (data == 'b') && (state == (NAV_TO_PILE || NAV_TO_BIN) )){
        ROS_WARN("Hit an object. Changing path.");
        abruptStop();
        
        ros::Rate reverse_wait(.75);
        sendMotorCmd(GO_BACKWARD, 70);
        reverse_wait.sleep();
        
        ros::Rate pivot_wait(1);
        sendMotorCmd(PIVOT_LEFT, 70);
        pivot_wait.sleep();
        
        sendMotorCmd(GO_FORWARD, 70);
        for (int i = 0; i < 10; i++)
        {
            ros::spinOnce();
            //avoid obstacle does two things:
            // 1) makes sure the robot doesn't hit anything
            // 2) Changes the state of the robot to re-find its target
            avoid_obstacle();
            loop_rate.sleep();
        }
    }
}

//Description: Updates global variable (x position of pile)
//Called by: SpinOnce()
void processColor(const robot::object::ConstPtr &msg)
{
    xpos_pile = msg->xpos;
    area_pile = msg->area;
}

//Description: updates global variables (xpos_bin and area_bin)
//Called by: spinOnce()
void processObject(const robot::object::ConstPtr &msg)
{
    xpos_bin = msg->xpos;
    area_bin = msg->area;
}
