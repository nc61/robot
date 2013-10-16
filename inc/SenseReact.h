#ifndef SENSE_REACT_H
#define SENSE_REACT_H

#include "robot/IR.h"

#define WANDER 0
#define MID_VN 5000
#define LEFT_VN 5000
#define RIGHT_VN 5000
#define MID_FAR 3000
#define LEFT_FAR 3000
#define RIGHT_FAR 3000
#define GO_BACKWARD 1
#define PIVOT_RIGHT 2
#define PIVOT_LEFT 3
#define GO_FORWARD 4
#define STOP_MOTORS 5


void sendCommand(uint8_t command, uint8_t duty_cycle);
void stop();
void avoid_obstacle();
void xmegaFeedback(const std_msgs::Char::ConstPtr &msg);
void processIR(const robot::IR::ConstPtr &msg);

#endif
