#include <string>
#include <iostream>
#include <stdlib.h>
#include "robot/IR.h"
#include "ros/ros.h"

void sensorCallback(const robot::IR::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MaestroSubscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("sensor_data", 1000, sensorCallback);
    ros::spin();
    
}

void sensorCallback(const robot::IR::ConstPtr &msg)
{
    uint16_t leftIR = msg->leftIR;
    uint16_t rightIR = msg->rightIR;
    ROS_INFO("Received data: Left IR: %d\tRight IR: %d", leftIR, rightIR);
}
