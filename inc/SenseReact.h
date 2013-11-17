#ifndef SENSE_REACT_H
#define SENSE_REACT_H

#include "robot/sensors.h"
#include "std_msgs/UInt8.h"

#define WANDER 0
#define MID_VN 175
#define LEFT_VN 375
#define RIGHT_VN 375
#define MID_FAR 160
#define LEFT_FAR 200
#define RIGHT_FAR 200
#define FSR_THRESHOLD 50
#define GO_BACKWARD 1
#define PIVOT_RIGHT 2
#define PIVOT_LEFT 3
#define GO_FORWARD 4
#define STOP_MOTORS 5

#define XPOS_PILE_LEFT_LIMIT 250
#define XPOS_PILE_RIGHT_LIMIT 350
#define AREA_PILE_THRESH 3000000
#define AREA_PILE_LOW 1000000
#define PILE_IR_THRESH 175 


void sendMotorCommand(uint8_t command, uint8_t duty_cycle);
void sendServoCommand(uint8_t command);
void stop();
void avoid_obstacle();
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg);
void processSensors(const robot::sensors::ConstPtr &msg);

#endif
