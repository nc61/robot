#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "Common.h"


using namespace cv;

uint8_t hue_low = 0;
uint8_t hue_high = 255;
uint8_t sat_low = 0; 
uint8_t sat_high = 255;
uint8_t val_low = 0;
uint8_t val_high = 255;

IplImage* GetThresholdedImage(IplImage *img)
{
    IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    cvCvtColor(img, imgHSV, CV_BGR2HSV);
    IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
    cvInRangeS(imgHSV, cvScalar(hue_low,sat_low,val_low), cvScalar(hue_high,sat_high,val_high), imgThreshed);
    cvReleaseImage(&imgHSV);
    return imgThreshed;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "calibrate");
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
        //ROS_INFO("Area: %f", area);
        //ROS_INFO("x position: %f", xpos);

        ROS_INFO("Hue: %d-%d\tSat: %d-%d\tVal: %d-%d\n", hue_low, hue_high, sat_low, sat_high, val_low, val_high);

        cvShowImage("Window", redTrack);
        char ch = cvWaitKey(10);
        switch (ch)
        {
            //hue
            case 'q':
                if (hue_low == hue_high)
                    ROS_INFO("hue_low must be less than hue_high");
                else 
                hue_low = hue_low + 1;
            break;
            case 'a':
                if (hue_low == 0) 
                    ROS_INFO("Hue is minimum");
                else
                    hue_low = hue_low - 1;
            break;
            case 'w':
                if (hue_high == 255)
                    ROS_INFO("Hue is maximum");
                else
                    hue_high = hue_high + 1;
            break;
            case 's':
                if (hue_high == hue_low)
                    ROS_INFO("hue_high must be greater than hue_low");
                else
                    hue_high = hue_high - 1;
            break;
           
            //saturation 
            case 'e':
                if (sat_low == sat_high)
                    ROS_INFO("sat_low must be less than sat_high");
                else 
                sat_low = sat_low + 1;
            break;
            case 'd':
                if (sat_low == 0) 
                    ROS_INFO("sat is minimum");
                else
                    sat_low = sat_low - 1;
            break;
            case 'r':
                if (sat_high == 255)
                    ROS_INFO("sat is maximum");
                else
                    sat_high = sat_high + 1;
            break;
            case 'f':
                if (sat_high == sat_low)
                    ROS_INFO("sat_high must be greater than sat_low");
                else
                    sat_high = sat_high - 1;
            break;
            
            //value 
            case 't':
                if (val_low == val_high)
                    ROS_INFO("val_low must be less than val_high");
                else 
                val_low = val_low + 1;
            break;
            case 'g':
                if (val_low == 0) 
                    ROS_INFO("val is minimum");
                else
                    val_low = val_low - 1;
            break;
            case 'y':
                if (val_high == 255)
                    ROS_INFO("val is maximum");
                else
                    val_high = val_high + 1;
            break;
            case 'h':
                if (val_high == val_low)
                    ROS_INFO("val_high must be greater than val_low");
                else
                    val_high = val_high - 1;
            break;
        }
    }

    free(moments);
    cvDestroyWindow("Window");
    cvReleaseCapture(&capture);
    return 0;
}
