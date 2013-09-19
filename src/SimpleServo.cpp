#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>

using namespace LibSerial;

float getPos();
SerialStream maestro;

int main(int argc, char** argv){
    
    ros::init(argc, argv, "lab3");
    ros::NodeHandle nh;
    
    maestro.Open("/dev/ttyACM0");

    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl;
        exit(1);
    }
    
    maestro.SetBaudRate(SerialStreamBuf::BAUD_57600);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set baud rate."
                  << std::endl;
        exit(1);
    }
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set char size."
                  << std::endl;
        exit(1);
    }
    maestro.SetNumOfStopBits(1);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set num stop bits."
                  << std::endl;
        exit(1);
    }
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set parity."
                  << std::endl;
        exit(1);
    }
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not set flow control."
                  << std::endl;
        exit(1);
    }
    
    char cmd_speed[] = {0x84, 0x04, 0xE4, 0x00};
    float position = 0;
    ros::Rate update_rate(20);
    
    while(ros::ok()){
        char cmd_max[] = {0x84, 0x04, 0x40, 0x3E};
        
        maestro.write(cmd_max, sizeof(cmd_max));
        
        while (!(abs(position - 45) < .1))
        {
            position = getPos();
            ROS_INFO("Current position: %f", getPos());
            update_rate.sleep();

        }
        
        char cmd_min[] = {0x84, 0x04, 0x68, 0x07};
        
        maestro.write(cmd_min, sizeof(cmd_min));
        
        while (!(abs(position + 45) < .1)) 
        {
            position = getPos();
            ROS_INFO("Current position: %f", getPos());
            update_rate.sleep();
        }

    }
    maestro.Close();
}

float getPos()
{
    char cmd[] = {0x90, 0x04};
    maestro.write(cmd, sizeof(cmd));

    char response[2];
    maestro.read(response, sizeof(response));

    float pulse_length = (response[0] + 256*response[1])/4000;
    float position = (90)*(pulse_length - 1.5);

    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Serial error in getPos()."
                  << std::endl;
        exit(1);
    }

    return position;
}
