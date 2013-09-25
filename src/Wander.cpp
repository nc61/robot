#include <string>
#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "robot/IR.h"

#define WANDER 0
#define OBS_AVOID 1
#define IR_THRESH 5000
#define OVER_ROTATE 1000
#define AVOID_BACKWARDS 1
#define PIVOT_RIGHT 2
#define PIVOT_LEFT 3
#define GO_FORWARD 4

uint8_t mode = WANDER;
uint8_t prev_mode = WANDER;
uint16_t rightIR;
uint16_t leftIR;
void processIR(const robot::IR::ConstPtr &msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Wander");
    ros::NodeHandle nh;
    ros::Publisher motorPub = nh.advertise<std_msgs::UInt8>("motor_command", 1000);
    ros::Subscriber IRSub = nh.subscribe<robot::IR>("sensor_data", 1000, processIR);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        switch (mode)
        {
            // Obstacle avoid mode. In this case,
            // the robot adjusts until there is no
            // longer an obstacle in the way, then it
            // reverts to the previous state.
            case OBS_AVOID:
            { // Need this bracket to establish a scope
              // for the msg
                std_msgs::UInt8 msg;
                
                if ((rightIR < IR_THRESH) && (leftIR < IR_THRESH))
                {
                    msg.data = AVOID_BACKWARDS;
                    motorPub.publish(msg);
                    ROS_INFO("Backing up and changing direction");
                }
                
                else if (leftIR < IR_THRESH)
                {
                    msg.data = PIVOT_RIGHT;
                    motorPub.publish(msg);
                    ROS_INFO("Pivoting right");
                    
                    // Motors turn left while IR values are above
                    // The threshold plus a little extra (to avoid
                    // having to avoid again immediately)
                    while (leftIR < IR_THRESH + OVER_ROTATE)
                    {                   
                        // Update IR values using the callback
                        // function
                        ros::spinOnce();         
                    }
                }
                
                else if (rightIR < IR_THRESH)
                {
                    msg.data = PIVOT_LEFT;
                    motorPub.publish(msg);
                    ROS_INFO("Pivoting left");
                    
                    while (rightIR < IR_THRESH - OVER_ROTATE)
                    {
                        ros::spinOnce();
                    }
                }
                mode = prev_mode; //Go back to the original mode
            }
        }

        case WANDER:
        {
            std_msgs::UInt8 msg;
            msg.data = GO_FORWARD;
            motorPub.publish(msg);
        }
        // Run loop at 10Hz
        loop_rate.sleep();
    }

}

void processIR(const robot::IR::ConstPtr &msg)
{
    rightIR = msg->rightIR;
    leftIR = msg->leftIR;
    
    if (mode != OBS_AVOID) 
    {
        if ((rightIR < IR_THRESH) || (leftIR < IR_THRESH))
        {
            prev_mode = mode;
            mode = OBS_AVOID;
        }
    }
}
