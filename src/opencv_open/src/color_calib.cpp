#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <stdio.h>
#include <opencv2/core.hpp>

using namespace cv;

const int max_value_H = 360/2;
const int max_value = 255;

const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
    Mat frame_HSV, frame_threshold, frame_blurred, frame_masked;

    cv_bridge::CvImagePtr cv_ptr;

    try 
    { 
        // Trackbars to set thresholds for HSV values
        createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
        createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
        createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
        createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
        createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
        createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);

        // Convert the ROS message  
        cv_ptr = cv_bridge::toCvCopy(imgMsg);
        
        // Store the values of the OpenCV compatible image into the frame
        Mat frame = cv_ptr->image;

        // Change the color from RGB to BGR, and them from BGR to HSV
        cvtColor(frame, frame, COLOR_RGB2BGR);  
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

        // Blur the image
        GaussianBlur(frame_HSV, frame_blurred, Size(5, 5), 0);

        Mat dElement = getStructuringElement( MORPH_RECT, Size( 15, 15 ), Point());
        Mat eElement = getStructuringElement( MORPH_RECT, Size( 15, 15 ), Point());

        // Dilate the image, making the white smaller
        dilate(frame_blurred, frame_blurred, dElement);

        // Erode the image, making it back the normal size
        erode(frame_blurred, frame_blurred, eElement);

        // Detect the object based on HSV Range Values
        inRange(frame_blurred, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_masked);

        // Show the frames
        imshow(window_capture_name, frame);
        imshow("tresh", frame_masked);

        waitKey(30);
    }// Try

    catch (cv_bridge::Exception& e) 
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgMsg->encoding.c_str());
      }
      
}// Void

int main(int argc, char* argv[])
{
    // The name of the node
    ros::init(argc, argv, "colors");
  
    // Default handler for nodes in ROS
    ros::NodeHandle nh;

    // Used to publish and subscribe to images
    image_transport::ImageTransport it(nh);

    // Subscribe to the /camera topic
    image_transport::Subscriber subImage = it.subscribe("/d400/color/image_raw", 1, imageCallback);
    namedWindow(window_detection_name);
    ros::Rate loop_rate(10);

    // Make sure we keep reading new video frames by calling the imageCallback function
    ros::spin();
}