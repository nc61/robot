#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "robot/IR.h"
#include "ros/ros.h"

#define SENSOR_IR_L  0
#define SENSOR_IR_R  1
#define SERVO_ARM_L  2
#define SERVO_ARM_R  3
#define SERVO_BUCKET 4


using namespace LibSerial;

/***************************************************************
 * Description: This node publishes on the topic sensor_data. It
 *  reads data from pin 0 of the mini Maestro and publishes the 
 *  data as an integet
 *************************"*************************************/

uint16_t readPin(unsigned int channel);
void servoCallback(const std_msgs::UInt8::ConstPtr &msg);
void liftDirt();

SerialStream maestro;

int main(int argc, char** argv){
    
    maestro.Open("/dev/ttyACM0");

    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl;
        exit(1);
    }
    
    maestro.SetBaudRate(SerialStreamBuf::BAUD_115200);
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    maestro.SetNumOfStopBits(1);
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_HARD);

    ros::init(argc, argv, "Maestro");
    ros::NodeHandle nh;
    
    ros::Publisher infraredSensor = nh.advertise<robot::IR>("sensor_data", 1000);
    ros::Subscriber servoControl = nh.subscribe<std_msgs::UInt8>("servo_command", 1000, servoCallback);
    
    ros::Rate loop_rate(10);
    
    while(ros::ok())
    {
        robot::IR msg;
        msg.leftIR = readPin(SENSOR_IR_L); //Reading ADC value on pin 0
        msg.rightIR = readPin(SENSOR_IR_R); //Reading ADC value on pin 0
        ROS_INFO("Left IR: %d\tRight IR: %d", msg.leftIR, msg.rightIR);
        infraredSensor.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    maestro.Close();
}

uint16_t readPin(unsigned int channel)
{

    char command[] = {0x90, channel};
    maestro.write(command, sizeof(command));
    
    char response[2];
    maestro.read(response, sizeof(response));

    return response[0] + 256*response[1];
}

void liftDirt()
{
    char cmd_speed_bucket[] = {0x87, SERVO_BUCKET, 0x0C, 0x01};
    maestro.write(cmd_speed_bucket, sizeof(cmd_speed_bucket));
    char cmd_pos_bucket[] = {0x84, SERVO_BUCKET, 0x70, 0x2E};
    maestro.write(cmd_pos_bucket, sizeof(cmd_pos_bucket));
    ROS_INFO("Lifting dirt");
}

void servoCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    uint8_t command = msg->data;
    ROS_INFO("Received data %d", command);
    if (command == 0){liftDirt();}
}

