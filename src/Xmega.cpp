#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "robot/motor.h"
#include <SerialStream.h>
#include "SenseReact.h"
#include "Common.h"
#include "Xmega.h"

using namespace LibSerial;
SerialStream xmega;

int main(int argc, char** argv)
{
    std::string str = "/dev/ttyUSB0";
    xmega.Open(str);
    xmega.SetBaudRate(SerialStreamBuf::BAUD_57600);
    xmega.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    xmega.SetNumOfStopBits(1);
    xmega.SetParity(SerialStreamBuf::PARITY_NONE);
    xmega.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!xmega.good())
    {
        std::cerr << "Error opening serial port "
                  << str
                  << std::endl;
        exit(1);
    }

    ros::init(argc, argv, "Xmega");
    ros::NodeHandle nh;
    ros::Rate init_delay(10);
    ros::Rate loop_rate(10);

    //Publishers
    ros::Publisher motorFeedback = nh.advertise<std_msgs::Char>("motor_feedback", 1000);
    init_delay.sleep();
    //Subscribers
    ros::Subscriber motorCommand = nh.subscribe<robot::motor>("motor_command", 1000, motorCallback);
    
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        if (xmega.rdbuf()->in_avail())
        {
            char byte;
            xmega.get(byte);
            ROS_INFO("Received byte: %d", byte);
            if (byte == 0xAA)
            {
                //Message 1 means motors have stopped
                std_msgs::Char msg;
                msg.data = 1;
                motorFeedback.publish(msg);
            }
        }
    }
}

void motorCallback(const robot::motor::ConstPtr &msg)
{
    uint8_t command = msg->command;
    uint8_t duty_cycle = msg->duty_cycle;
    
    if (command == GO_FORWARD)
    {
        ROS_INFO("Going forward");
        char cmd[] = {'m', 'b', 'f', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    } 
    else if (command == PIVOT_RIGHT)
    {
        ROS_INFO("Pivoting right");
        char cmd[] = {'p', 'r', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == PIVOT_LEFT)
    {
        ROS_INFO("Pivoting left");
        char cmd[] = {'p', 'l', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == GO_BACKWARD)
    {
        ROS_INFO("Going backward");
        char cmd[] = {'m', 'b', 'b', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == STOP_MOTORS)
    {
        ROS_INFO("Stopping");
        char cmd[] = {'s', 'y'};
        xmega.write(cmd, sizeof(cmd));
    }
}
