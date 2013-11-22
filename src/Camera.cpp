#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "ros/ros.h"
#include "Camera.h"
#include "Common.h"
#include <ctime>

ros::Publisher objectRecPub;
ros::Publisher colorPub;
uint8_t state = FIND_BIN;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "Camera");
    ros::NodeHandle nh;
    
    //Publisher
    objectRecPub = nh.advertise<robot::object>("object_data", 1);
    colorPub = nh.advertise<robot::color>("color_data", 1);
   
    //Subscriber
    ros::Subscriber stateSub = nh.subscribe<std_msgs::UInt8>("state_data", 1, processState);

    cv::Mat frame;
    cv::Mat img_scene, img_object;
    cv::Mat hsvFrame, bgrFrame;
    
    img_object = cv::imread("/home/nick/img/dew.bmp", CV_LOAD_IMAGE_GRAYSCALE );
    
    if(!img_object.data)
    { 
        ROS_FATAL("Error reading object image");
        return -1; 
    }
       
    //Initialize a detector and an extractor
    cv::SurfFeatureDetector detector(MIN_HESSIAN);
    cv::SurfDescriptorExtractor extractor;
    
    //Calculate and extract key points of the object
    std::vector<cv::KeyPoint> keypoints_object;
    detector.detect(img_object, keypoints_object);
    cv::Mat descriptors_object;
    extractor.compute(img_object, keypoints_object, descriptors_object);

    //Check to make sure descriptors are calculated
    if (descriptors_object.empty() ) 
    {
        ROS_FATAL("Couldn't calculate descriptors for object");
        return -1;
    }
    
    //Find the corners of the object image
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); 
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows); 
    obj_corners[3] = cvPoint(0, img_object.rows);

    
    //Open up video stream
    cv::VideoCapture capture(1);
    if (!capture.isOpened())
    {
        ROS_FATAL("Error opening camera");
        return(-1);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    
    while(ros::ok()){
        
        if (state == FIND_BIN || state == NAV_TO_BIN)
        {
            //Capture frame, then extract key points and descriptors
            capture >> frame;
            
            cv::cvtColor(frame, img_scene, CV_RGB2GRAY);
            if(!img_scene.data)
            {
                ROS_FATAL("Eror creating scene matrix\n");
                return(-1);
            }
            
            std::vector<cv::KeyPoint> keypoints_scene;
            detector.detect(img_scene, keypoints_scene);
            cv::Mat descriptors_scene;
            
            extractor.compute(img_scene, keypoints_scene, descriptors_scene);
            if (!descriptors_scene.empty())
            {
            
                //Match descriptor vectors using FLANN matcher
                cv::FlannBasedMatcher matcher;
                std::vector<cv::DMatch> matches;
                matcher.match(descriptors_object, descriptors_scene, matches);
                
//                ROS_INFO("time: %ld", time(NULL));
                //Calculate minimum and maximum distances    
                double max_dist = 0; 
                double min_dist = 100;
                for(int i = 0; i < descriptors_object.rows; i++)
                { 
                    double dist = matches[i].distance;
                    if(dist < min_dist) min_dist = dist;
                    if(dist > max_dist) max_dist = dist;
                }
                
                //Draw only good matches
                std::vector<cv::DMatch> good_matches;

                for( int i = 0; i < descriptors_object.rows; i++ )
                { 
                    if(matches[i].distance <= .4*max_dist) 
                        good_matches.push_back( matches[i]);
                }
                
                //Localize the object
                std::vector<cv::Point2f> obj;
                std::vector<cv::Point2f> scene;

                if (good_matches.size() >= 4)
                { 
                    for( int i = 0; i < good_matches.size(); i++ )
                    {   //Get the keypoints from the good matches
                        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
                        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt );
                    }
                    
                    cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );

                    //Get the corners from the object image 
                    std::vector<cv::Point2f> scene_corners(4);
                    cv::perspectiveTransform(obj_corners, scene_corners, H);

         /*           
                    line( img_scene, scene_corners[0], 
                          scene_corners[1], Scalar(0, 255, 0), 4 );
                    line( img_scene, scene_corners[1], 
                            scene_corners[2], Scalar( 0, 255, 0), 4 );
                    line( img_scene, scene_corners[2], 
                            scene_corners[3], Scalar( 0, 255, 0), 4 );
                    line( img_scene, scene_corners[3], 
                            scene_corners[0], Scalar( 0, 255, 0), 4 );
            */
                   
                    float xCenter = (scene_corners[0].x + scene_corners[2].x)/2;
                    float area = abs(scene_corners[0].x - scene_corners[2].x)*abs(scene_corners[0].y - scene_corners[3].y);


                    robot::object msg;
                    msg.x0 = scene_corners[0].x;
                    msg.x1 = scene_corners[1].x;
                    msg.x2 = scene_corners[2].x;
                    msg.x3 = scene_corners[3].x;

                    msg.y0 = scene_corners[0].y;
                    msg.y1 = scene_corners[1].y;
                    msg.y2 = scene_corners[2].y;
                    msg.y3 = scene_corners[3].y;

                    msg.xpos = xCenter;
                    msg.area = area;
                    objectRecPub.publish(msg);
                    //                   cv::imshow("object", img_scene);
                    //                   cvWaitKey(1);
                }
            }
        } else if (state == FIND_PILE || state == NAV_TO_PILE) {
            
            capture >> bgrFrame;
            cv::blur(bgrFrame, bgrFrame, cv::Size(BLUR_SIZE));
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
    //        cv::imshow("color", hsvFrame);
     //       cvWaitKey(1);
        }
        ros::spinOnce();
    }
    return 0;
}

void processState(const std_msgs::UInt8::ConstPtr &msg)
{
    state = msg->data;
}

