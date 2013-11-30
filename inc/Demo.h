#ifndef DEMO_H
#define DEMO_H

#include "robot/sensors.h"
#include "robot/color.h"
#include "robot/object.h"
#include "std_msgs/UInt8.h"

#define NORMAL_SPEED 65
#define FAST 70
#define SLOW_PIVOT_TO_PILE 63
#define IR_BIN_RIGHT 300 
#define IR_BIN_LEFT 300
#define IR_BIN_MID 350
#define IR_BIN_DUMP 500 

#define TILT_BACK_WAIT_TIME 1500
#define LIFT_DIRT_WAIT_TIME 1800
#define REVERSE_FROM_PILE_WAIT_TIME 1500
#define REVERSE_FROM_DUMP_WAIT_TIME 1500
#define PREPARE_DUMP_WAIT_TIME 1000
#define DUMP_DIRT_WAIT_TIME 2200
#define PIVOT_FROM_BIN_WAIT_TIME 2000

#define CENTER_PILE_PIVOT_TIME 60
#define CENTER_PILE_CYCLES 16
#define NTB_FWD_TIME 300
#define ALIGN_BIN_PIVOT_TIME 60
#define BIN_FOUND_PIVOT_TIME 60
#define BIN_FOUND_CYCLES 20
#define BIN_CENTER_PIVOT_TIME 60 
#define BIN_NOT_FOUND_CYCLES 20
#define BIN_NOT_SEEN_PIVOT_TIME 200
#define AVOID_MOVE_FWD_TIME 500

void sendMotorCommand(uint8_t command, uint8_t duty_cycle);
void sendServoCommand(uint8_t command);
void stop();
int avoid_obstacle();
void xmegaFeedback(const std_msgs::UInt8::ConstPtr &msg);
void processSensors(const robot::sensors::ConstPtr &msg);
void processColor(const robot::color::ConstPtr &msg);
void processObject(const robot::object::ConstPtr &msg);
void delayms(uint16_t ms, uint8_t avoid);
void delayms(uint16_t msg);
void findPile();
void navToPile();
void navToBin();
void findBin();
void smallMovement(uint8_t direction, uint16_t interval, uint8_t avoid);
void smallMovement(uint8_t direction, uint8_t duty_cycle, uint16_t interval, uint8_t avoid);


#endif
