#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <image_geometry/pinhole_camera_model.h>
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
using namespace aruco;

class CameraInfoImageNode {
public:

    Mat distCoeffs;
    Mat cameraMatrix = (Mat_<float>(3,3) << 606, 0, 342, 0, 605, 241, 0, 0, 1);
    image_geometry::PinholeCameraModel cameraModel;
    opencv_open::DetectionArray detections;
    visualization_msgs::Marker cube;

    CameraInfoImageNode() {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Subscribe to the camera info topic
        camera_info_subscriber = nh.subscribe("/camera_info", 1, &CameraInfoImageNode::cameraInfoCallback, this);

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber image_subscriber = it.subscribe("/image_raw", 1, &CameraInfoImageNode::imageCallback, this);

        detections_pub = nh.advertise<opencv_open::DetectionArray>("/aruco_detection", 10);
        marker_pub = nh.advertise<visualization_msgs::Marker>("/detection_marker", 10);           
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        cameraModel.fromCameraInfo(camera_info_msg);
        distCoeffs = cameraModel.distortionCoeffs();
        ROS_INFO("Received camera info!");
    }

    Vec3f rotationMatrixToEulerAngles(Mat &R){
        float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        bool singular = sy < 1e-6;
    
        float x, y, z;
        if (!singular){
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
        cv_bridge::CvImagePtr cv_image_ptr;
        try {
            cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat frame = cv_image_ptr->image;
        Mat frame_BGR;

        // Change the color from RGB to BGR
        cvtColor(frame, frame_BGR, COLOR_RGB2BGR);

        // Create a vector for the ids, corners and rejected candidates
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners, rejectedCandidates;
        float markerLength = 0.06;

        //Set default parameters
        Ptr<DetectorParameters> parameters = DetectorParameters::create();
        Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_50);

        // Detect the markers
        detectMarkers(frame_BGR, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // Clone the frame_BGR into a new matrix
        Mat frame_aruco = frame_BGR.clone();

        // Draw a contour and the id of the markers
        drawDetectedMarkers(frame_aruco, markerCorners, markerIds);
        Mat frame_aruco_pose = frame_aruco.clone();
        Mat objPoints(4, 1, CV_32FC3);
        vector<Vec3d> rvecs, tvecs;
        Mat rotationMat;

        // Takes the camera matrix and the distortion coefficient
        Mat camMat = this -> cameraMatrix;
        Mat distCoeffic = this -> distCoeffs;

        for(int i = 0; i < markerIds.size(); i++){
            // Takes the rotation and translation vector of each marker
            estimatePoseSingleMarkers(markerCorners, markerLength, camMat, distCoeffic, rvecs, tvecs, objPoints);
            
            // Draw on the image the axis of each marker
            drawFrameAxes(frame_aruco_pose, camMat, distCoeffic, rvecs[i], tvecs[i], markerLength);
            
            // Transform the vector in a matrix
            Rodrigues(rvecs[i], rotationMat);

            // Transform the rotation matrix to euler angles
            Vec3f angles = rotationMatrixToEulerAngles(rotationMat);

            // Post on the topic, on this mesage, the id of the camera
            cube.header.frame_id = "camera_color_optical_frame";
            cube.pose.position.x = tvecs[i](0) + (0.03 * sin(angles[1]));
            cube.pose.position.y = tvecs[i](1);
            cube.pose.position.z = tvecs[i](2) + (0.03 * cos(angles[1]));
            cube.pose.orientation = geometry_msgs::Quaternion(tf::createQuaternionMsgFromRollPitchYaw(angles[0], angles[1], angles[2]));
            cube.color.a = 0.8;
            cube.color.r = 1;
            cube.color.g = 0;
            cube.color.b = 1;
            cube.scale.x = 0.06;
            cube.scale.y = 0.06;
            cube.scale.z = 0.06;
            cube.id = int(markerIds[i]);
            cube.lifetime.fromSec(1/30);
            cube.type = cube.CUBE;

            opencv_open::Detection detect;
            detect.id = markerIds[i];
            detect.pose = cube.pose;
            detect.header = cube.header;
            detect.name = "aruco_cube_" + markerIds[i];
            //cout << "Detect: "<< detect << endl;
            detections.header.stamp = ros::Time::now();
            detections.detections.push_back(detect);
            //cout << "Detections: " << detections << endl;

            tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z));
            tf::Quaternion q(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w);
            //tf::Matrix3x3 m(q);
            //double roll, pitch, yaw;
            //m.getRPY(roll, pitch, yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), detect.name, "d400_color_optical_frame"));

            marker_pub.publish(cube);
        }
        
        imshow("window", frame_aruco_pose);
        waitKey(30);
    }

private:
    ros::Subscriber camera_info_subscriber;
    ros::Publisher detections_pub;
    ros::Publisher marker_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_info_image_node");
    CameraInfoImageNode node;
    ros::spin();
    return 0;
}