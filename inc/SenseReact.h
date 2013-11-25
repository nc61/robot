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
#define FSR_THRESHOLD 60
#define GO_BACKWARD 1
#define PIVOT_RIGHT 2
#define PIVOT_LEFT 3
#define GO_FORWARD 4
#define STOP_MOTORS 5
#define PIVOT_LEFT_IMM 6
#define PIVOT_RIGHT_IMM 7
#define STOP_IMMEDIATELY 8
#define SET_LEFT_MOTOR 9
#define SET_RIGHT_MOTOR 10
#define GO_FORWARD_IMM 11

#define XPOS_PILE_LEFT_LIMIT 100 
#define XPOS_PILE_RIGHT_LIMIT 220 
#define XPOS_PILE_CENTERED 160 
#define AREA_PILE_THRESH 80000
#define AREA_PILE_LOW 60000
#define AREA_PILE_CLOSE 400000

#define XPOS_BIN_LEFT_LIMIT 150 
#define XPOS_BIN_CENTERED 160
#define XPOS_BIN_RIGHT_LIMIT 170
#define YPOS_BIN_CLOSE 80
#define AREA_BIN_THRESH 5000
#define AREA_BIN_LOW 100
#define AREA_BIN_CLOSE 25000
#define ORIENTATION_THRESHOLD 30
#define XPOS_BIN_ALIGN_RIGHT 80
#define XPOS_BIN_ALIGN_LEFT 240 
#define NTB_FWD_CYCLES 20

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3

#define AVOID 1
#define NO_AVOID 0

#endif
