#ifndef COMMON_H
#define COMMON_H
#include "robot/color.h"
#include "robot/object.h"
#include "robot/motor.h"
#include "robot/sensors.h"
#define LOOP_RATE 20 
#define INIT_DELAY 1

#define LIFT_DIRT    0
#define DUMP_DIRT    1
#define LOWER_BUCKET 2
#define SERVO_INIT   3
#define DIG_BUCKET   4
#define TILT_BACK_BUCKET 5

#define DONE_LIFTING  0
#define DONE_DUMPING  1
#define DONE_LOWERING 2

#define FIND_PILE 0
#define NAV_TO_PILE 1
#define APPROACH_PILE 2
#define FIND_BIN 3
#define NAV_TO_BIN 4
#define APPROACH_BIN 5
#define WANDER 6

#endif
