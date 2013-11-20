#ifndef SENSE_REACT_H
#define SENSE_REACT_H

#include "robot/sensors.h"
#include "robot/color.h"
#include "robot/object.h"
#include "std_msgs/UInt8.h"

#define MID_VN 250 
#define LEFT_VN 250
#define RIGHT_VN 250
#define MID_FAR 200
#define MID_VF 125
#define LEFT_FAR 200
#define RIGHT_FAR 200
#define FSR_THRESHOLD 150
#define GO_BACKWARD 1
#define PIVOT_RIGHT 2
#define PIVOT_LEFT 3
#define GO_FORWARD 4
#define STOP_MOTORS 5

#define XPOS_PILE_LEFT_LIMIT 285 
#define XPOS_PILE_RIGHT_LIMIT 315
#define AREA_PILE_THRESH 600000
#define AREA_PILE_LOW 100000
#define AREA_PILE_CLOSE 8000000
#define PILE_IR_THRESH 175 

#define XPOS_BIN_LEFT_LIMIT 140 
#define XPOS_BIN_RIGHT_LIMIT 160
#define AREA_BIN_THRESH 5000
#define AREA_BIN_LOW 100
#define AREA_BIN_CLOSE 80000
#define BIN_IR_THRESH 175 


void sendMotorCommand(uint8_t command, uint8_t duty_cycle);
void sendServoCommand(uint8_t command);
void stop();
void avoid_obstacle();
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg);
void processSensors(const robot::sensors::ConstPtr &msg);
void processColor(const robot::color::ConstPtr &msg);
void processObject(const robot::object::ConstPtr &msg);

#endif
