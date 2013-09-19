#include <SerialStream.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include "std_msgs/UInt16.h"
#include "ros/ros.h"

using namespace LibSerial;

/***************************************************************
 * Description: This node publishes on the topic sensor_data. It
 *  reads data from pin 0 of the mini Maestro and publishes the 
 *  data as an integet
 ***************************************************************/

uint16_t readPin(unsigned int channel);
SerialStream maestro;

int main(int argc, char** argv){
    
    int analogValue;

    maestro.Open("/dev/ttyACM0");

    if (!maestro.good())
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port."
                  << std::endl ;
        exit(1) ;
    }
    
    maestro.SetBaudRate(SerialStreamBuf::BAUD_115200);
    maestro.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    maestro.SetNumOfStopBits(1);
    maestro.SetParity(SerialStreamBuf::PARITY_NONE);
    maestro.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_HARD);

    ros::init(argc, argv, "Maestro");
    ros::NodeHandle nh;
    ros::Publisher infraredSensor = nh.advertise<std_msgs::UInt16>("sensor_data", 1000);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std_msgs::UInt16 msg;
        msg.data = readPin(0); //Reading ADC value on pin 0
        ROS_INFO("%d", msg.data);
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


