#ifndef DEMO_H
#define DEMO_H

#include "robot/sensors.h"
#include "robot/color.h"
#include "robot/object.h"
#include "std_msgs/UInt8.h"

void sendMotorCommand(uint8_t command, uint8_t duty_cycle);
void sendServoCommand(uint8_t command);
void stop();
void avoid_obstacle();
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg);
void processSensors(const robot::sensors::ConstPtr &msg);
void processColor(const robot::color::ConstPtr &msg);
void processObject(const robot::object::ConstPtr &msg);
void delayms(uint16_t ms);
void navToBin();
void findBin(uint8_t dir);


#endif
