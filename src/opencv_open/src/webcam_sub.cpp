#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <boost/bind.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core.hpp>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <opencv_open/DetectionArray.h>
#include <opencv_open/Detection.h>

using namespace std;
using namespace cv;

ros::Publisher marker_pub;
ros::Publisher detections_pub;

class c{
  private:
    tf::TransformBroadcaster br;

  public:

    opencv_open::DetectionArray detections;
    visualization_msgs::Marker cube;

    // Create the matrix that will be used to store the image
    Mat frame_HSV, frame_blurred, frame_threshold[4], frame_with_contours[4];

    // Create an array with the values for each color in HSV
    int low_H[4]{0, 20, 60, 100};
    int low_S[4]{160, 180, 100, 110};
    int low_V[4]{105, 165, 40, 50};

    int high_H[4]{8, 40, 90, 120};
    int high_S[4]{255, 255, 255, 255};
    int high_V[4]{255, 255, 255, 255};

    // Create a string with the name of the windows
    const String windows[4]
    {
      "red_detection", "yellow_detection", "green_detection", "blue_detection"
    };

    Mat depthFrame;

    // It takes the image and the cordinates of a pixel on it, returning the x, y and z of the object on that pixel
    void pixelToCordinates(Mat depthF, int x, int y, float coord[3])
    {
      //cout << x << " | " << y << endl;
      double invFx = 1/606.1556396484375;
      double invFy = 1/605.013427734375;
      double cx = 341.7103271484375;
      double cy = 241.36676025390625;
     
      ushort zValue = depthF.at<ushort>(y, x);
      coord[2] = zValue * 0.001;
      coord[0] = (x - cx) * coord[2] * invFx;
      coord[1] = (y - cy) * coord[2] * invFy;
      //cout << "blue" << endl << "X: " << xMetric << endl << "Y: " << yMetric << endl << "Z: " << zMetric << endl;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& depthMsg)
    {
      // Pointer used for the conversion from a ROS message to an OpenCV compatible image
      cv_bridge::CvImagePtr depth_cv_ptr;

      // Convert the ROS message
      depth_cv_ptr = cv_bridge::toCvCopy(depthMsg);

      // Store the values of the OpenCV compatible image into the frame
      depthFrame = depth_cv_ptr->image;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
    {
      // Pointer used for the conversion from a ROS message to an OpenCV compatible image
      cv_bridge::CvImagePtr cv_ptr;

      try 
      { 
        // Convert the ROS message  
        cv_ptr = cv_bridge::toCvCopy(imgMsg);
        
        // Store the values of the OpenCV compatible image into the frame
        Mat current_frame = cv_ptr->image;

        // Change the color from RGB to BGR, and them from BGR to HSV
        cvtColor(current_frame, current_frame, COLOR_RGB2BGR);  
        cvtColor(current_frame, frame_HSV, COLOR_BGR2HSV);

        // Blur the image
        GaussianBlur(frame_HSV, frame_blurred, Size(5, 5), 0);
        
        // Add the color filter/mask
        // Red - 0
        inRange(frame_blurred, Scalar(low_H[0], low_S[0], low_V[0]), Scalar(high_H[0], high_S[0], high_V[0]), frame_threshold[0]);
        // Yellow - 1
        inRange(frame_blurred, Scalar(low_H[1], low_S[1], low_V[1]), Scalar(high_H[1], high_S[1], high_V[1]), frame_threshold[1]);
        // Green - 2
        inRange(frame_blurred, Scalar(low_H[2], low_S[2], low_V[2]), Scalar(high_H[2], high_S[2], high_V[2]), frame_threshold[2]);
        // Blue - 3
        inRange(frame_blurred, Scalar(low_H[3], low_S[3], low_V[3]), Scalar(high_H[3], high_S[3], high_V[3]), frame_threshold[3]);

        // Kernel used on dilate and erode
        Mat dElement = getStructuringElement( MORPH_RECT, Size( 15, 15 ), Point());
        Mat eElement = getStructuringElement( MORPH_RECT, Size( 15, 15 ), Point());

        // Dilate the image, making the white smaller
        dilate(frame_threshold[0], frame_threshold[0], dElement);
        dilate(frame_threshold[1], frame_threshold[1], dElement);
        dilate(frame_threshold[2], frame_threshold[2], dElement);
        dilate(frame_threshold[3], frame_threshold[3], dElement);

        // Erode the image, making it back the normal size
        erode(frame_threshold[0], frame_threshold[0], eElement);
        erode(frame_threshold[1], frame_threshold[1], eElement);
        erode(frame_threshold[2], frame_threshold[2], eElement);
        erode(frame_threshold[3], frame_threshold[3], eElement);

        // Create a vector for the contours and another for the hierarchy
        vector<vector<Point>> contoursRed;
        vector<vector<Point>> contoursYellow;
        vector<vector<Point>> contoursGreen;
        vector<vector<Point>> contoursBlue;
        vector<Vec4i> hierarchy;

        Mat frameDepth = this -> depthFrame;

        // Make a reference to the mesages Marker as cube
        visualization_msgs::Marker cube;
        /**/
        // Add the contours (aomosso)
        // Red - 0
        findContours(frame_threshold[0], contoursRed, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        frame_with_contours[0] = Mat::zeros(frame_threshold[0].size(), CV_8UC3);

        for( size_t i = 0; i< contoursRed.size(); i++ )
        {
          Scalar color = Scalar(0, 255, 0);
          drawContours(frame_with_contours[0], contoursRed, (int)i, color, 2, LINE_8, hierarchy, 0);
          Rect redRect = boundingRect(contoursRed[i]);
          float wRedRect = redRect.width;
          float hRedRect = redRect.height;
          float xRedRect = redRect.x;
          float yRedRect = redRect.y;
          float coordRed[3] = {0, 0, 0};
          pixelToCordinates(frameDepth, int(xRedRect+wRedRect/2), int(yRedRect+hRedRect/2), coordRed);

          // Publish the cordinates
          if (coordRed[0] !=0 and coordRed[2] < 2 and contourArea(contoursRed[i]) > 100)
          {
            cout << "Red " << i << " X: " << coordRed[0] << " Y: " << coordRed[1] << " Z: " << coordRed[2] << endl;
            circle(frame_with_contours[0], Point(int(xRedRect+wRedRect/2), int(yRedRect+hRedRect/2)), 2, (0,0,255), -1);
            cube.header.frame_id = "d400_color_optical_frame";
            cube.pose.position.x = coordRed[0];
            cube.pose.position.y = coordRed[1];
            cube.pose.position.z = coordRed[2] + 0.03;

            cube.color.a = 0.5;
            cube.color.r = 1;

            cube.scale.x = 0.06;
            cube.scale.y = 0.06;
            cube.scale.z = 0.06;

            cube.id = int(contoursRed.size());
            cube.lifetime.fromSec(1/30);
            cube.type = cube.CUBE;
            cube.pose.orientation.w = 1;

            opencv_open::Detection detect;
            detect.id = int(contoursRed.size());
            detect.pose = cube.pose;
            detect.header = cube.header;
            detect.name = "Red";
            cout << "Detect: "<< detect << endl;
            detections.header.stamp = ros::Time::now();
            detections.detections.push_back(detect);
            //cout << "Detections: " << detections << endl;

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z));
            tf::Quaternion q(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w);
            //tf::Matrix3x3 m(q);
            //double roll, pitch, yaw;
            //m.getRPY(roll, pitch, yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "d400_color_optical_frame", "Red"));
            marker_pub.publish(cube);
          }
          detections_pub.publish(detections);
        }
        imshow(windows[0], frame_with_contours[0]);
        
        // Yellow - 1
        findContours(frame_threshold[1], contoursYellow, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        frame_with_contours[1] = Mat::zeros(frame_threshold[1].size(), CV_8UC3);

        for( size_t i = 0; i< contoursYellow.size(); i++ )
        {
          Scalar color = Scalar(0, 255, 0);
          drawContours(frame_with_contours[1], contoursYellow, (int)i, color, 2, LINE_8, hierarchy, 0);
          Rect yellowRect = boundingRect(contoursYellow[i]);
          float wYellowRect = yellowRect.width;
          float hYellowRect = yellowRect.height;
          float xYellowRect = yellowRect.x;
          float yYellowRect = yellowRect.y;
          float coordYellow[3] = {0, 0, 0};
          pixelToCordinates(frameDepth, int(xYellowRect+wYellowRect/2), int(yYellowRect+hYellowRect/2), coordYellow);

          if (coordYellow[0] !=0 and coordYellow[2] < 2 and contourArea(contoursYellow[i]) > 100)
          {
            cout << "Yellow " << i << " X: " << coordYellow[0] << " Y: " << coordYellow[1] << " Z: " << coordYellow[2] << endl;
            circle(frame_with_contours[1], Point(int(xYellowRect+wYellowRect/2), int(yYellowRect+hYellowRect/2)), 2, (0,0,255), -1);
            cube.header.frame_id = "d400_color_optical_frame";
            cube.pose.position.x = coordYellow[0];
            cube.pose.position.y = coordYellow[1];
            cube.pose.position.z = coordYellow[2] + 0.03;

            cube.color.a = 0.5;
            cube.color.r = 1;

            cube.scale.x = 0.06;
            cube.scale.y = 0.06;
            cube.scale.z = 0.06;

            cube.id = int(contoursYellow.size());
            cube.lifetime.fromSec(1/30);
            cube.type = cube.CUBE;
            cube.pose.orientation.w=1;

            opencv_open::Detection detect;
            detect.id = int(contoursYellow.size());
            detect.pose = cube.pose;
            detect.header = cube.header;
            detect.name = "Yellow";
            cout << "Detect: "<< detect << endl;
            detections.header.stamp = ros::Time::now();
            detections.detections.push_back(detect);
            //cout << "Detections: " << detections << endl;

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z));
            tf::Quaternion q(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w);
            //tf::Matrix3x3 m(q);
            //double roll, pitch, yaw;
            //m.getRPY(roll, pitch, yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "d400_color_optical_frame", "Yellow"));
            marker_pub.publish(cube);
          }
          detections_pub.publish(detections);
        }
        imshow(windows[1], frame_with_contours[1]);
        
        // Green - 2
        findContours(frame_threshold[2], contoursGreen, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        frame_with_contours[2] = Mat::zeros(frame_threshold[2].size(), CV_8UC3);

        for( size_t i = 0; i< contoursGreen.size(); i++ )
        {
          Scalar color = Scalar(0, 255, 0);
          drawContours(frame_with_contours[2], contoursGreen, (int)i, color, 2, LINE_8, hierarchy, 0);
          Rect greenRect = boundingRect(contoursGreen[i]);
          float wGreenRect = greenRect.width;
          float hGreenRect = greenRect.height;
          float xGreenRect = greenRect.x;
          float yGreenRect = greenRect.y;
          float coordGreen[3] = {0, 0, 0};
          pixelToCordinates(frameDepth, int(xGreenRect+wGreenRect/2), int(yGreenRect+hGreenRect/2), coordGreen);

          if (coordGreen[0] !=0 and coordGreen[2] < 2 and contourArea(contoursGreen[i]) > 100)
          {
            cout << "Green " << i << " X: " << coordGreen[0] << " Y: " << coordGreen[1] << " Z: " << coordGreen[2] << endl;
            circle(frame_with_contours[2], Point(int(xGreenRect+wGreenRect/2), int(yGreenRect+hGreenRect/2)), 2, (0,0,255), -1);
            cube.header.frame_id = "d400_color_optical_frame";
            cube.pose.position.x = coordGreen[0];
            cube.pose.position.y = coordGreen[1];
            cube.pose.position.z = coordGreen[2] + 0.03;

            cube.color.a = 0.5;
            cube.color.r = 1;

            cube.scale.x = 0.06;
            cube.scale.y = 0.06;
            cube.scale.z = 0.06;

            cube.id = int(contoursGreen.size());
            cube.lifetime.fromSec(1/30);
            cube.type = cube.CUBE;
            cube.pose.orientation.w=1;

            opencv_open::Detection detect;
            detect.id = int(contoursGreen.size());
            detect.pose = cube.pose;
            detect.header = cube.header;
            detect.name = "Green";
            cout << "Detect: "<< detect << endl;
            detections.header.stamp = ros::Time::now();
            detections.detections.push_back(detect);
            //cout << "Detections: " << detections << endl;

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z));
            tf::Quaternion q(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w);
            //tf::Matrix3x3 m(q);
            //double roll, pitch, yaw;
            //m.getRPY(roll, pitch, yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "d400_color_optical_frame", "Green"));
            marker_pub.publish(cube);
          }
          detections_pub.publish(detections);
        }
        imshow(windows[2], frame_with_contours[2]);
        
        // Blue - 3
        findContours(frame_threshold[3], contoursBlue, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        frame_with_contours[3] = Mat::zeros(frame_threshold[3].size(), CV_8UC3);

        for( size_t i = 0; i< contoursBlue.size(); i++ )
        {
          Scalar color = Scalar(0, 255, 0);
          
          drawContours(frame_with_contours[3], contoursBlue, (int)i, color, 2, LINE_8, hierarchy, 0);
          Rect blueRect = boundingRect(contoursBlue[i]);
          float wBlueRect = blueRect.width;
          float hBlueRect = blueRect.height;
          float xBlueRect = blueRect.x;
          float yBlueRect = blueRect.y;
          float coordBlue[3] = {0, 0, 0};
          pixelToCordinates(frameDepth, int(xBlueRect+wBlueRect/2), int(yBlueRect+hBlueRect/2), coordBlue);

          //putText(frame_with_contours[3], to_string(xBlue), Point(10, frame_with_contours[0].rows / 2), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
          //putText(frame_with_contours[3], to_string(yBlue), Point(10, frame_with_contours[0].rows / 3), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 255), 2);
          //putText(frame_with_contours[3], to_string(zBlue), Point(10, frame_with_contours[0].rows / 4), FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 255), 2);

          if (coordBlue[0] !=0 and coordBlue[2] < 2 and contourArea(contoursBlue[i]) > 100)
          {
            cout << "blue " << i << " X: " << coordBlue[0] << " Y: " << coordBlue[1] << " Z: " << coordBlue[2] << endl;
            //cout << "Size: " << contourArea(contoursBlue[1])
            circle(frame_with_contours[3], Point(int(xBlueRect+wBlueRect/2), int(yBlueRect+hBlueRect/2)), 2, (0,0,255), -1);
            cube.header.frame_id = "d400_color_optical_frame";
            cube.pose.position.x = coordBlue[0];
            cube.pose.position.y = coordBlue[1];
            cube.pose.position.z = coordBlue[2] + 0.03;

            cube.color.a = 0.5;
            cube.color.r = 1;

            cube.scale.x = 0.06;
            cube.scale.y = 0.06;
            cube.scale.z = 0.06;

            cube.id = int(contoursBlue.size());
            cube.lifetime.fromSec(1/30);
            cube.type = cube.CUBE;
            cube.pose.orientation.w=1;

            opencv_open::Detection detect;
            detect.id = int(contoursBlue.size());
            detect.pose = cube.pose;
            detect.header = cube.header;
            detect.name = "Blue";
            cout << "Detect: "<< detect << endl;
            detections.header.stamp = ros::Time::now();
            detections.detections.push_back(detect);
            //cout << "Detections: " << detections << endl;

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z));
            tf::Quaternion q(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w);
            //tf::Matrix3x3 m(q);
            //double roll, pitch, yaw;
            //m.getRPY(roll, pitch, yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "d400_color_optical_frame", "Blue"));
            marker_pub.publish(cube);
        
          }

          detections_pub.publish(detections);
        }

        imshow(windows[3], frame_with_contours[3]);
        imshow("tresh", frame_threshold[3]);
        // Display frame for 30 milliseconds
        waitKey(30);

      }// Try

      catch (cv_bridge::Exception& e) 
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgMsg->encoding.c_str());
      }

    }// Void
};// Class
 
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "colors");
  
  c mainClass;

  // Default handler for nodes in ROS
  ros::NodeHandle n;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(n);
  image_transport::ImageTransport ti(n);

  // Subscribe to the /camera topic
  image_transport::Subscriber subImage = it.subscribe("/d400/color/image_raw", 1, &c::imageCallback, &mainClass);
  image_transport::Subscriber depthSub = ti.subscribe("/d400/aligned_depth_to_color/image_raw", 1, &c::depthCallback, &mainClass);

  detections_pub = n.advertise<opencv_open::DetectionArray>("color_detection", 10);
  marker_pub = n.advertise<visualization_msgs::Marker>("color_detection_marker", 10);

  ros::Rate loop_rate(10);

  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();

  return 0;
}