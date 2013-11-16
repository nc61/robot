#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;

/** @function main */
int main( int argc, char** argv )
{
    Mat frame;
    Mat grayImage;
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        std::cout << "Error opening camera";
        return(-1);
    }

    while(1){
        capture >> frame;
        cvtColor(frame, grayImage, CV_RGB2GRAY);
        imshow("GrayScale", grayImage);
        if (waitKey('e') > 0) break;
    }

    cvReleaseCapture(&capture);
    return 0;
}

  /** @function readme */
