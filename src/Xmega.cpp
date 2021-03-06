#include "ros/ros.h"
#include "std_msgs/UInt8.h"
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
    ros::Rate loop_rate(LOOP_RATE);
    ros::Rate init_delay(INIT_DELAY);

    //Publishers
    ros::Publisher xmegaFeedback = nh.advertise<std_msgs::UInt8>("xmega_feedback", 1000);
    init_delay.sleep();
    //Subscribers
    ros::Subscriber motorCommand = nh.subscribe<robot::motor>("motor_command", 1000, motorCallback);
    ros::Subscriver controllerCommand = nh.subscribe<robot::controller>("controller", 1, controllerCallback)
    init_delay.sleep();
    
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        if (xmega.rdbuf()->in_avail())
        {
            char byte;
            xmega.get(byte);
            ROS_INFO("Received char: %c", byte);
            if (byte == 's')
            {
                //Message 1 means motors have stopped
                std_msgs::UInt8 msg;
                msg.data = 1;
                ROS_INFO("Sending message %d", msg.data);
                xmegaFeedback.publish(msg);
            }
        }
    }

    ROS_INFO("Stopping");
    char cmd_stop_motors[] = {'s', 'n'};
    xmega.write(cmd_stop_motors, sizeof(cmd_stop_motors));

}

void controllerCallback(const robot::controller::ConstPtr &msg)
{
    uint8_t left_duty_cycle = msg->left_duty_cycle;
    uint8_t left_direction = msg->left_direction;
    uint8_t right_duty_cycle = msg->right_duty_cycle;
    uint8_t right_direction = msg->right_direction;

    char cmd[] = {'c', left_duty_cycle, left_direction, right_duty_cycle, right_direction};
    xmega.write(cmd, sizeof(cmd));
}

void motorCallback(const robot::motor::ConstPtr &msg)
{
    uint8_t command = msg->command;
    uint8_t duty_cycle = msg->duty_cycle;
    
    if (command == GO_FORWARD)
    {
        char cmd[] = {'m', 'b', 'f', duty_cycle, 'n'};
        xmega.write(cmd, sizeof(cmd));
    } 
    else if (command == PIVOT_RIGHT)
    {
        char cmd[] = {'p', 'r', duty_cycle, 'n'};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == PIVOT_LEFT)
    {
        char cmd[] = {'p', 'l', duty_cycle, 'n'};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == GO_BACKWARD)
    {
        char cmd[] = {'m', 'b', 'b', duty_cycle, 'n'};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == STOP_MOTORS)
    {
        char cmd[] = {'s', 'y'};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == STOP_IMMEDIATELY)
    {
        char cmd[] = {'i'};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == PIVOT_LEFT_IMM)
    {
        char cmd[] = {'p', 'l', duty_cycle, 'y'};
        xmega.write(cmd, sizeof(cmd));
    }
    else if (command == PIVOT_RIGHT_IMM)
    {
        char cmd[] = {'p', 'r', duty_cycle, 'y'};
        xmega.write(cmd, sizeof(cmd));
    }
	else if (command == SET_LEFT_MOTOR)
	{
		char cmd[] = {'m', 'l', 'f', duty_cycle, 'n'};
        	xmega.write(cmd, sizeof(cmd));
	}
	else if (command == SET_RIGHT_MOTOR)
	{
		char cmd[] = {'m', 'r', 'f',duty_cycle, 'n'};
        	xmega.write(cmd, sizeof(cmd));
	}
    else if (command == GO_FORWARD_IMM)
    {
        char cmd[] = {'m', 'b', 'f', duty_cycle, 'y'};
        xmega.write(cmd, sizeof(cmd));
    }
}
