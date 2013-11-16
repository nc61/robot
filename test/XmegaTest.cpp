#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "robot/motor.h"
#include "Common.h"

void feedbackCallback(const std_msgs::Char::ConstPtr &msg);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "xmega");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
  
    //Publisher
    ros::Publisher motorCommand = nh.advertise<robot::motor>("motor_command", 1000);
    loop_rate.sleep();

    //Subscriber
    ros::Subscriber motorFeedback = nh.subscribe<std_msgs::Char>("motor_feedback", 1000, feedbackCallback);
   
    robot::motor msg;
    while(ros::ok()){
        msg.command = 's';
        motorCommand.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void feedbackCallback(const std_msgs::Char::ConstPtr &msg)
{
    char data = msg->data;
    ROS_INFO("Received feedback: %x", data);
}

