#include "ros/ros.h"
#include <stdlib.h>
#include "robot/motor.h"
#include "robot/sensors.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"
#include "robot/object.h"
#include "SenseReact.h"
#include "Common.h"
#include "Demo.h"

//IR information and states
uint16_t rightIR;
uint16_t midIR;
uint16_t leftIR;
uint8_t FSR;

float xpos_bin,y_avg_bin,x_0,x_1,x_2,x_3,y_0,y_1,y_2,y_3;
float area_bin, area_bin_prev;
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
	ros::Rate delay_rate(100);
	sendMotorCommand(PIVOT_LEFT_IMM, 75);
	for (int i = 0; i < 5; i++) { delay_rate.sleep(); }
	
        if (area_bin > AREA_BIN_THRESH && (xpos_bin > 125))
        {
            ROS_INFO("Area > AREA_BIN_THRESH (area = %f, xpos = %f)", area_bin, xpos_bin);
            while (y_avg_bin < 125) 
            {
                sendMotorCommand(GO_FORWARD_IMM, 70);
                for (int i = 0; i < 20; i++) { loop_rate.sleep(); }
                ROS_INFO("y_avg: %f", y_avg_bin); 
                sendMotorCommand(STOP_IMMEDIATELY, 0);
                for (int i = 0; i < 20; i++) { loop_rate.sleep(); }
                ros::spinOnce(); 
            }
            stop();
            if (y_3 - y_2 > 10)
            {
                ROS_INFO("y3 > y2");
                while (xpos_bin > 40) { 
                    ros::Rate delay_rate(100);
                    sendMotorCommand(PIVOT_RIGHT_IMM, 75);
                    for (int i = 0; i < 5; i++) delay_rate.sleep(); 
                    sendMotorCommand(STOP_IMMEDIATELY, 0);
                    ROS_INFO("x_pos: %f", xpos_bin); 
                    for (int i = 0; i < 40; i++) delay_rate.sleep(); 
                    ros::spinOnce(); 
                }
                sendMotorCommand(GO_FORWARD, 75);
                while (leftIR < 275) { ros::spinOnce(); loop_rate.sleep(); }
                stop();
                sendMotorCommand(PIVOT_LEFT, 75);
                while (midIR < 315) { ros::spinOnce(); loop_rate.sleep(); }
                stop();
                return 0;
            } else if (y_2 - y_3 > 10) {
                ROS_INFO("y3 > y2");
                while (xpos_bin < 260) 
                { 
                    ros::Rate delay_rate(100);
                    sendMotorCommand(PIVOT_LEFT_IMM, 75);
                    for (int i = 0; i < 5; i++) delay_rate.sleep(); 
                    sendMotorCommand(STOP_IMMEDIATELY, 0);
                    ROS_INFO("x_pos: %f", xpos_bin); 
                    for (int i = 0; i < 40; i++) delay_rate.sleep(); 
                    ros::spinOnce(); 
                }
                sendMotorCommand(GO_FORWARD, 75);
                while (rightIR < 275) { ros::spinOnce(); loop_rate.sleep(); }
                stop();
                sendMotorCommand(PIVOT_RIGHT, 75);
                while (midIR < 315) { ros::spinOnce(); loop_rate.sleep(); }
                stop();
                return 0;
            }

        } else { 
	        ROS_INFO("No sight of pile");
            sendMotorCommand(STOP_IMMEDIATELY, 0);
            for (int i = 0; i < 40; i++) { delay_rate.sleep(); }
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
    y_avg_bin = (msg->y2 + msg->y3)/2;
    y_3 = msg->y3;
    y_2 = msg->y2;
    y_1 = msg->y1;
    y_0 = msg->y0;
    xpos_bin = msg->xpos;
	area_bin_prev = area_bin;
    area_bin = msg->area;
}

