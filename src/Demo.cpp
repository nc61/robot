#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/IR.h"
#include "robot/tracking.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "SenseReact.h"
#include "Common.h"

//IR information and states
uint8_t mode = FIND_PILE;
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
char wait;
double xpos_pile, area_pile;

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
    ros::Subscriber IRSub = nh.subscribe<robot::IR>("sensor_data", 2, processIR);
    ros::Subscriber xmegaSub = nh.subscribe<std_msgs::UInt8>("xmega_feedback", 1, xmegaFeedback);
    ros::Subscriber colorSub = nh.subscribe<robot::tracking>("color_data", 1, processColor);

    while(ros::ok())
    {
        //In wander mode, robot moves forward with a duty cycle of 75
        if (mode == FIND_PILE)
        {
            ROS_INFO("Finding pile");
            //Pivot right until color is detected, then move forward
            sendMotorCmd(PIVOT_RIGHT, 70);
            while ( (abs(xpos_pile - PILE_CENTERED) > PILE_XPOS_THRESH) || (area_pile < PILE_AREA_THRESH) )
                ros::spinOnce();
            sendMotorCmd(GO_FORWARD, 70);
            mode = NAV_TO_PILE;
        } else if (mode == NAV_TO_PILE) {
            ROS_INFO("Navigating to pile");
            while ( midIR < PILE_IR_THRESHOLD && )
            {
                if (leftIR > LEFT_VN || rightIR > RIGHT_VN)
                    avoid_obstacle();
                ros::spinOnce();
                loop_rate.sleep();
            }
                
            mode = APPROACH_PILE;
            //TO DO
        } else if (mode == APPROACH_PILE) {
            ROS_INFO("Approaching pile");
            sendMotorCmd(GO_FORWARD, 50);
            sendServoCmd(DIG_BUCKET);
            while (FSR_weight < FSR_threshold){ ros::spinOnce(); }
            sendMotorCmd(LIFT_DIRT);
            stop();
            mode = FIND_BIN;
        } else if (mode == FIND_BIN) {
            ROS_INFO("Finding bin");
            //TO DO
        } else if (mode == APPROACH_BIN) {
            ROS_INFO("Approaching bin");
            sendMotorCmd(GO_FORWARD, 50);
            while (midIR < BUCKET_CLOSE){ ros::spinOnce() };
            stop();
            sendServoCmd(DROP_DIRT);
            sendServoCmd(RAISE_BUCKET_HIGH);
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
        stop();
        sendMotorCmd(GO_BACKWARD, 70);
        while(midIR > MID_FAR) { ros::spinOnce(); }
        stop();
        (leftIR > rightIR) ? sendMotorCmd(PIVOT_RIGHT, 70) : sendMotorCmd(PIVOT_LEFT, 70) ; 
        while (midIR > MID_FAR){ ros::spinOnce(); }
        stop();
    } else if (leftIR > LEFT_VN) {
        stop();
        ROS_WARN("Object to left");
        sendMotorCmd(PIVOT_RIGHT, 70);
        while (leftIR > LEFT_FAR) { ros::spinOnce(); }
        stop();
    } else if (rightIR > RIGHT_VN) {
        stop();
        ROS_WARN("Object to right");
        sendMotorCmd(PIVOT_LEFT, 70);
        while (rightIR > RIGHT_FAR) { ros::spinOnce(); }
        stop();
    }
    if (mode == NAV_TO_PILE)
        mode = FIND_PILE;
    else if (mode == NAV_TO_BIN)
        mode = FIND_BIN;
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

void abruptStop()
{
    robot::motor msg;
    msg.command = STOP_MOTORS_INSTANTLY;
    msg.duty_cycle = 0;
    motorPub.publish(msg);
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
void processIR(const robot::IR::ConstPtr &msg)
{
    leftIR = msg->leftIR;
    midIR = msg->midIR;
    rightIR = msg->rightIR;
}

//Description: updates global variable (FSR_weight)
//Called by: spinOnce()
void processFSR(const std_msgs::UInt8::ConstPtr &msg)
{
    FSR_weight = msg->data;
}

//Description: Updates global variable (feedback from xmega)
//Called by: spinOnce()
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg)
{
    xmega_feedback = msg->data;
    //A 'reflex' response to the bump switch
    if ( (data == 'b') && (mode == (NAV_TO_PILE || NAV_TO_BIN) )){
        ROS_WARN("Hit an object. Changing path.");
        abruptStop();
        
        ros::Rate reverse_wait(.75);
        sendMotorCmd(GO_BACKWARD, 70);
        reverse_wait.sleep();
        
        ros::Rate pivot_wait(1);
        sendMotorCmd(PIVOT_LEFT, 50);
        pivot_wait.sleep();
        
        sendMotorCmd(GO_FORWARD, 70);
        for (int i = 0; i < 10; i++)
        {
            ros::spinOnce();
            //avoid obstacle does two things:
            // 1) makes sure the robot doesn't hit anything
            // 2) Changes the mode of the robot to re-find its target
            avoid_obstacle();
            loop_rate.sleep();
        }
    }
}

//Description: Updates global variable (x position of pile)
//Called by: SpinOnce()
void processColor(const robot::tracking::ConstPtr &msg)
{
    xpos_pile[i % 4] = msg->xpos;
    area_pile[i % 4] = msg->area;
    ROS_INFO("x Position of pile: %f\tArea of pile: %f\n", xpos_pile);
}
