#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/sensors.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "robot/object.h"
#include "SenseReact.h"
#include "Common.h"

//IR information and states
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
uint8_t FSR;

float xpos_bin;
float area_bin;
char wait;

//Establish global scope for publisher
ros::Publisher motorPub;
ros::Publisher servoPub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "NavToColor");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    //Publisher
    motorPub = nh.advertise<robot::motor>("motor_command", 1);
    servoPub = nh.advertise<std_msgs::UInt8>("servo_command", 1);
    loop_rate.sleep();
    //Subscriber
    ros::Subscriber maestroSub = nh.subscribe<robot::sensors>("sensor_data", 2, processSensors);
    ros::Subscriber xmegaSub = nh.subscribe<std_msgs::UInt8>("xmega_feedback", 1, xmegaFeedback);
    ros::Subscriber objectSub = nh.subscribe<robot::object>("object_data", 1, processObject);

    //initialize servos
    sendServoCommand(SERVO_INIT);

    while(ros::ok())
    {
        if (area_bin > AREA_BIN_THRESH)
        {
            ROS_INFO("Area > area_bin_THRESH (area = %f, xpos = %f)", area_bin, xpos_bin);

            if (xpos_bin < XPOS_BIN_LEFT_LIMIT)
            {
                ROS_INFO("xpos: %f", xpos_bin); 
                sendMotorCommand(PIVOT_LEFT, 65);
                while (xpos_bin < XPOS_BIN_LEFT_LIMIT)
                {
                    ROS_INFO("Pivoting left (xpos = %f)", xpos_bin);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            } else if (xpos_bin > XPOS_BIN_RIGHT_LIMIT) {
                ROS_INFO("xpos: %f", xpos_bin); 
                stop();
                sendMotorCommand(PIVOT_RIGHT, 65);
                while (xpos_bin > XPOS_BIN_RIGHT_LIMIT)
                {
                    ROS_INFO("Pivoting left (xpos = %f)", xpos_bin);
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }

            sendMotorCommand(GO_FORWARD, 65);
            while (area_bin < AREA_BIN_CLOSE)
            { 
                ros::spinOnce();
                loop_rate.sleep();
            }
            ROS_INFO("Near bin");
            stop();
            return 0;

        } else { 
	        ROS_INFO("No sight of pile");
            sendMotorCommand(PIVOT_LEFT, 65);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}

//Description: avoids an obstacle
//Calls: stop(), sendMotorCommand(), avoid_obstacle()
void avoid_obstacle()
{
    if (midIR > MID_VN)
    {
        ROS_WARN("Object in front");
        sendMotorCommand(GO_BACKWARD, 65);
        while(midIR > MID_FAR) { ros::spinOnce(); }
        sendMotorCommand(PIVOT_RIGHT, 65);
        while(midIR > MID_VF) { ros::spinOnce(); }
    } else if (leftIR > LEFT_VN) {
        ROS_WARN("Object to left");
        sendMotorCommand(PIVOT_RIGHT, 65);
        while (leftIR > LEFT_FAR) { ros::spinOnce(); }
    } else if (rightIR > RIGHT_VN) {
        ROS_WARN("Object to right");
        sendMotorCommand(PIVOT_LEFT, 65);
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
void sendMotorCommand(uint8_t command, uint8_t duty_cycle)
{
    robot::motor msg;
    msg.command = command;
    msg.duty_cycle = duty_cycle;
    motorPub.publish(msg);
}

//Description: Sends a command to the Maestro servo controller
void sendServoCommand(uint8_t command)
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
//Called by: spinOnce();
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg)
{
    wait = msg->data;
    ROS_INFO("wait value: %d", wait);
}

void processObject(const robot::object::ConstPtr &msg)
{
    xpos_bin = msg->xpos;
    area_bin = msg->area;
}

