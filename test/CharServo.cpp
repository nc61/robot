#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <ros/ros.h>

using namespace LibSerial;

float getPos();
SerialStream maestro;
void getError(uint8_t location);

int main(int argc, char** argv){
    
    ros::init(argc, argv, "CharServo");
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
   
    char cmd_init[] = {0x84, 0x04, 0x70, 0x2E};
    maestro.write(cmd_init, sizeof(cmd_init));
    getError(0);
    
    char cmd_speed[] = {0x87, 0x04, 0x10, 0x00};
    maestro.write(cmd_speed, sizeof(cmd_speed));
    getError(1);
    
    while(ros::ok())
    {
        uint16_t pulse_time;
        uint8_t cmd_high;
        uint8_t cmd_low;
        
        std::cout << "Enter a pulse time: ";
        std::cin >> pulse_time; 
        cmd_high = ((pulse_time*4) >> 7) & 0x7F;
        cmd_low = ((pulse_time*4) & 0x7F);
        std::cout << "cmd[] = {0x84, 0x04, " << cmd_low << ", " << cmd_high << "};";
        
        char cmd[] = {0x84, 0x04, cmd_low, cmd_high};
        maestro.write(cmd, sizeof(cmd));
    }
    
    maestro.write(cmd_init, sizeof(cmd_init));
    maestro.Close();
}


void getError(uint8_t location)
{
    char cmd_error[] = {0xA1};
    maestro.write(cmd_error, sizeof(cmd_error));

    char error_char[2];
    maestro.read(error_char, sizeof(error_char));

    uint8_t error_num[2];
    error_num[0] = error_char[0];
    error_num[1] = error_char[1];

    ROS_INFO("Error code %x%x at location %d", error_num[0], error_num[1], location);
}

float getPos()
{
    char cmd[] = {0x90, 0x04};
    maestro.write(cmd, sizeof(cmd));

    char response[2];
    maestro.read(response, sizeof(response));

    float pulse_length = float((response[0] + 256*response[1]))/4000;
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
