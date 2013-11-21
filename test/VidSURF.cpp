#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "robot/object.h"
#include "ros/ros.h"
#include <ctime>

ros::Publisher objectRecPub;

int main( int argc, char** argv )
{
   ros::init(argc, argv, "VidSURF");
   ros::NodeHandle nh;
   objectRecPub = nh.advertise<robot::object>("object_data", 1);

    int minHessian = 500;
    Mat frame;
    Mat img_scene;
    Mat img_object;
    img_object = imread("/home/nick/img/dew.bmp", CV_LOAD_IMAGE_GRAYSCALE );
    
    if(!img_object.data)
    { 
        std::cout << "Error reading object image" << std::endl; 
        return -1; 
    }
    
    VideoCapture capture(1);
    if (!capture.isOpened())
    {
        std::cout << "Error opening camera" << std::endl;
        return(-1);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
       
    //Initialize a detector and an extractor
    SurfFeatureDetector detector(minHessian);
    SurfDescriptorExtractor extractor;
    
    //Calculate and extract key points of the object
    std::vector<KeyPoint> keypoints_object;
    detector.detect(img_object, keypoints_object);
    Mat descriptors_object;
    extractor.compute(img_object, keypoints_object, descriptors_object);

    //Check to make sure descriptors are calculated
    if (descriptors_object.empty() ) 
        cvError(0,"MatchFinder","1st descriptor empty",__FILE__,__LINE__); 
    
    //Find the corners of the object image
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); 
    obj_corners[1] = cvPoint(img_object.cols, 0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows); 
    obj_corners[3] = cvPoint(0, img_object.rows);

    while(1){

        //Capture frame, then extract key points and descriptors
        capture >> frame;
        cvtColor(frame, img_scene, CV_RGB2GRAY);

        if(!img_scene.data)
        {
            std::cout << "Error creating scene matrix" << std::endl;
            return(-1);
        }
        
        std::vector<KeyPoint> keypoints_scene;
        detector.detect(img_scene, keypoints_scene);
        Mat descriptors_scene;
        
        extractor.compute(img_scene, keypoints_scene, descriptors_scene);
        if (!descriptors_scene.empty())
        {
        
            //Match descriptor vectors using FLANN matcher
            FlannBasedMatcher matcher;
            std::vector<DMatch> matches;
            matcher.match(descriptors_object, descriptors_scene, matches);
            
            std::cout << "time: " << time(NULL)  << std::endl;
            //Calculate minimum and maximum distances    
            double max_dist = 0; 
            double min_dist = 100;
            for(int i = 0; i < descriptors_object.rows; i++)
            { 
                double dist = matches[i].distance;
                if(dist < min_dist) min_dist = dist;
                if(dist > max_dist) max_dist = dist;
            }
            
            //Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector<DMatch> good_matches;

            for( int i = 0; i < descriptors_object.rows; i++ )
            { 
                if(matches[i].distance <= .4*max_dist) 
                    {good_matches.push_back( matches[i]);}
            }

            
            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            if (good_matches.size() >= 4)
            { 
                for( int i = 0; i < good_matches.size(); i++ )
                {
                    //Get the keypoints from the good matches
                    obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
                    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt );
                }
                
                Mat H = findHomography( obj, scene, CV_RANSAC );

                //Get the corners from the image_1 ( the object to be "detected" )
                std::vector<Point2f> scene_corners(4);
                perspectiveTransform(obj_corners, scene_corners, H);

                //Publish the x coordinate of the center of the box;
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
                //-- Show detected matches
               float xCenter = (scene_corners[0].x + scene_corners[2].x)/2;
               float area = abs(scene_corners[0].x - scene_corners[2].x)*abs(scene_corners[0].y - scene_corners[3].y);

               robot::object msg;
               msg.xpos = xCenter;
               msg.area = area;
               objectRecPub.publish(msg);
//               imshow("Object detection", img_scene); 

            } else {
 //              imshow("Object detection", img_scene);
            }
        } else {
  //             imshow("Object detection", img_scene);
        }
   //     char ch =  waitKey(10);
    //    if (ch == 27) break;
    }
    return 0;
}

  /** @function readme */
