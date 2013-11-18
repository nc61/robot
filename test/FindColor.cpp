#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/UInt8.h"
#include "robot/color.h"
#include "Common.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define HSV_MIN 153,97,134
#define HSV_MAX 176,219,249

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FindColor");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE);
    //Publisher
    ros::Publisher colorPub = nh.advertise<robot::color>("color_data", 1);
    loop_rate.sleep();

    cv::VideoCapture capture(1);
    if (!capture.isOpened())
    {
        ROS_FATAL("Camera failed to open.");
        return -1;
    }

    capture.set(CV_CAP_PROP_FRAME_WIDTH, 160);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 120);

    cv::Mat bgrFrame;
    cv::Mat hsvFrame;

    while(ros::ok())
    {
        capture >> bgrFrame;
        cv::blur(bgrFrame, bgrFrame, cv::Size(3,3));
        cv::cvtColor(bgrFrame, hsvFrame, CV_BGR2HSV);
        cv::inRange(hsvFrame, cv::Scalar(HSV_MIN), cv::Scalar(HSV_MAX), hsvFrame);
        cv::Moments colorMoments;
        colorMoments = cv::moments(hsvFrame);
        float xMoment = colorMoments.m10;
        float area = colorMoments.m00;
        float xCenter = xMoment/area;

        robot::color msg;
        msg.xpos = xCenter;
        msg.area = area;
        colorPub.publish(msg);
    }
}
