#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "robot/motor.h"
#include <SerialStream.h>

#define GO_FORWARD 0
#define GO_BACKWARDS 1
#define PIVOT_RIGHT 2
#define PIVOT_LEFT 3
#define STOP_MOTORS 4
#define DEBUG 5

void motorCallback(const robot::motor::ConstPtr &msg);

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
        std::cerr << "Error opening serial port"
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
                //Message 0 means motors have stopped
                std_msgs::Char msg;
                msg.data = 0;
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
        ROS_INFO("Going Forward");
        char cmd[] = {'m', 'b', 'f', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == PIVOT_RIGHT)
    {
        char cmd[] = {'p', 'r', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == PIVOT_LEFT)
    {
        char cmd[] = {'p', 'l', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == GO_BACKWARDS)
    {
        char cmd[] = {'m', 'b', 'b', duty_cycle};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == STOP_MOTORS)
    {
        char cmd[] = {'s', 'y'};
        xmega.write(cmd, sizeof(cmd));
    }
}
