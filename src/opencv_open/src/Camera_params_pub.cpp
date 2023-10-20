#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
/*
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
   
  try { 
   
    // Convert the ROS message  
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    Mat current_frame = cv_ptr->image;
    Mat gray_frame;
    
    Size patternsize(7,7);
    vector<Point2f> corners;
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;
    Size winSize (7,7);
    Size zeroZone (-1,-1);
    TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    vector<Point3f> obj;
    for(int j=0;j<49;j++)
      obj.push_back(Point3f(j/7, j%7, 0.0f));

    cvtColor(current_frame, gray_frame, COLOR_BGR2GRAY);

    bool found = findChessboardCorners(gray_frame, patternsize, corners);

    cornerSubPix(gray_frame, corners, winSize, zeroZone, criteria);

    drawChessboardCorners(gray_frame, patternsize, corners, found);

    imagePoints.push_back(corners);
    objectPoints.push_back(obj);
  
    Mat cameraMatrix = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector <Mat> rvecs, tvecs;

    cameraMatrix.ptr<float>(0)[0] = 1;
    cameraMatrix.ptr<float>(1)[1] = 1;

    calibrateCamera(objectPoints, imagePoints,  gray_frame.size(),  cameraMatrix,  distCoeffs, rvecs, tvecs);

    cout << cameraMatrix;
    cout << distCoeffs;
    //cout << rvecs;
    //cout << tvecs;

    imshow("window", gray_frame);

    waitKey(30);

  }//Try

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}//Void
*/
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "Camera_params_pub");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  //image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);
   
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  //cv::destroyWindow("window");
}