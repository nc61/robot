#include <string>
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "robot/IR.h"
#include "ros/ros.h"
#include "robot/Maestro.h"
#include "robot/SerialCom.hpp"


/***************************************************************
 * Description: This node continously reads IR sensors at 10Hz while
 *      receiving and interpreting servo commands.
 *************************"*************************************/

SerialCom maestro;

int main(int argc, char** argv){
    
    maestro.init("/dev/ttyACM0"); 
    ros::init(argc, argv, "Maestro");
    ros::NodeHandle nh;
    
    ros::Publisher infraredSensor = nh.advertise<robot::IR>("sensor_data", 1000);
    ros::Subscriber servoControl = nh.subscribe<std_msgs::UInt8>("servo_command", 1000, servoCallback);
    
    ros::Rate loop_rate(10);

    servoInit();
    
    while(ros::ok())
    {
        robot::IR msg;
        msg.leftIR = readPin(SENSOR_IR_L_PIN); //Reading ADC value on pin 0
        msg.rightIR = readPin(SENSOR_IR_R_PIN); //Reading ADC value on pin 0
        ROS_INFO("Left IR: %d\tRight IR: %d", msg.leftIR, msg.rightIR);
        infraredSensor.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

uint16_t readPin(unsigned int channel)
{

    char command[] = {0x90, channel};
    maestro.serialWrite(command);
    
    char* response;
    response = maestro.serialRead(2);

    return response[0] + 256*response[1];
}

void liftDirt()
{
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_LIFT_POS & 0x7F), 
                              ((SERVO_BUCKET_LIFT_POS << 7) & 0x7F) };
    
    maestro.serialWrite(cmd_pos_bucket);
    ROS_INFO("Lifting dirt");
}

void raiseBucket()
{
    char cmd_pos_arms[] =     {0x9F, 2, SERVO_ARM_L_PIN, 
                              (SERVO_ARM_RAISE_POS & 0x7F), 
                              ((SERVO_ARM_RAISE_POS << 7) & 0x7F), 
                              (SERVO_ARM_RAISE_POS & 0x7F), 
                              ((SERVO_ARM_RAISE_POS << 7) & 0x7F), 
                              };
    
    maestro.serialWrite(cmd_pos_arms);
    ROS_INFO("Raising bucket");
    
}

void servoInit()
{
    char cmd_speed_larm[] =   {0x87, SERVO_ARM_L_PIN, 
                              (SERVO_ARM_SPEED & 0x7F), 
                              ((SERVO_ARM_SPEED << 7) & 0x7F)
                              };
    
    maestro.serialWrite(cmd_speed_larm);
    
    char cmd_speed_rarm[] =   {0x87, SERVO_ARM_R_PIN, 
                              (SERVO_ARM_SPEED & 0x7F), 
                              ((SERVO_ARM_SPEED << 7) & 0x7F)
                              };
    
    maestro.serialWrite(cmd_speed_rarm);
    
    char cmd_pos_arms[] =     {0x9F, 2, SERVO_ARM_L_PIN,
                              (SERVO_ARM_BASE_POS & 0x7F), 
                              ((SERVO_ARM_BASE_POS << 7) & 0x7F), 
                              (SERVO_ARM_BASE_POS & 0x7F), 
                              ((SERVO_ARM_BASE_POS << 7) & 0x7F), 
                              };
    
    maestro.serialWrite(cmd_pos_arms);
    
    char cmd_speed_bucket[] = {0x87, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_SPEED & 0x7F), 
                              ((SERVO_BUCKET_SPEED << 7) & 0x7F) };
    
    maestro.serialWrite(cmd_speed_bucket);
    
    char cmd_pos_bucket[] =   {0x84, SERVO_BUCKET_PIN, 
                              (SERVO_BUCKET_BASE_POS & 0x7F), 
                              ((SERVO_BUCKET_BASE_POS << 7) & 0x7F) };
    
    maestro.serialWrite(cmd_pos_bucket);

}

void servoCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    uint8_t command = msg->data;
    ROS_INFO("Received data %d", command);
    
    if (command == 0){liftDirt();}
    else if (command == 1){raiseBucket();}
    else ROS_INFO("%d is not a valid command", command);
}

