#ifndef CAMERA_H
#define CAMERA_H

#include "std_msgs/UInt8.h"

#define HSV_MIN 153,97,140
#define HSV_MAX 176,219,255
#define MIN_HESSIAN 500
#define BLUR_SIZE 3,3

void processState(const std_msgs::UInt8::ConstPtr &msg);
#endif
