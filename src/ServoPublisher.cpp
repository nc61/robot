#include <string>
#include <iostream>
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "ros/ros.h"

#define SERVO_BUCKET_PICK_UP 0

int main(int argc, char** argv){

    ros::init(argc, argv, "ServoPublisher");
    ros::NodeHandle nh;
    ros::Publisher ServoCommand = nh.advertise<std_msgs::UInt8>("servo_command", 1000);

    std_msgs::UInt8 msg;
    msg.data = SERVO_BUCKET_PICK_UP;
    ROS_INFO("Sending servo command %d", msg.data);
    ServoCommand.publish(msg);
}



