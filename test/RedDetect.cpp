#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "Common.h"


using namespace cv;

IplImage* GetThresholdedImage(IplImage *img)
{
    IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    cvCvtColor(img, imgHSV, CV_BGR2HSV);
    IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
    cvInRangeS(imgHSV, cvScalar(0,100,100), cvScalar(10,255,255), imgThreshed);
    cvReleaseImage(&imgHSV);
    return imgThreshed;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "openCV");
    ros::NodeHandle nh;
    ros::Publisher colorPub = nh.advertise<std_msgs::Float32>("color_data", 3);
    ros::Rate loop_rate(LOOP_RATE);

    CvCapture* capture = cvCaptureFromCAM(1);
    IplImage* image = 0;
    IplImage* redTrack = 0;

    float moment10;
    float area;
    float xpos;
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));

    while(ros::ok())
    {
        //Capture fram
        image = cvQueryFrame(capture);

        //Remove all colors but those in range
        redTrack = GetThresholdedImage(image);
       
        //Calculate the moments, area, x-position
        cvMoments(redTrack, moments, 1);
        moment10 = cvGetSpatialMoment(moments, 1, 0);
        area = cvGetCentralMoment(moments, 0, 0);
        xpos = moment10/area;
        ROS_INFO("Area: %f", area);
        ROS_INFO("x position: %f", xpos);

        std_msgs::Float32 msg;
        msg.data = xpos;
        colorPub.publish(msg);

        cvShowImage("Window", redTrack);
        int ch = cvWaitKey(1);
        loop_rate.sleep();
    }

    free(moments);
    cvDestroyWindow("Window");
    cvReleaseCapture(&capture);
    return 0;
}
