#!/usr/bin/env python3.8
from __future__ import print_function

import roslib
roslib.load_manifest('opencv_open')
import sys
import rospy
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server
from opencv_open.cfg import ColorsConfig
from opencv_open.msg import Detection, DetectionArray
from opencv_open.srv import DetectionsOrganized, DetectionsOrganizedResponse
from std_srvs.srv import Empty, EmptyResponse

class cube_finder:
    def __init__(self):

        #Subscribers
        self.red_cubes_sub = rospy.Subscriber("/red_color_detection", DetectionArray, self.callback_red)
        self.green_cubes_sub = rospy.Subscriber("/green_color_detection", DetectionArray, self.callback_green)
        self.blue_cubes_sub = rospy.Subscriber("/blue_color_detection", DetectionArray, self.callback_blue)
        self.yellow_cubes_sub = rospy.Subscriber("/yellow_color_detection", DetectionArray, self.callback_yellow)
        self.aruco_cubes_sub = rospy.Subscriber("/aruco_detection", DetectionArray, self.callback_aruco)

        #Serviços
        self.srv = rospy.Service("/get_all_detections_organized", DetectionsOrganized, self.getOrganizedList)
        self.srvClean = rospy.Service("/list_cleanner", Empty, self.srvCallback)
        self.global_detections_list = {}
    
    def callback_red(self, red_data):
        for detection in red_data.detections:
            self.global_detections_list[detection.name] = detection
    
    def callback_green(self, red_data):
        for detection in red_data.detections:
            self.global_detections_list[detection.name] = detection
    
    def callback_blue(self, red_data):
        for detection in red_data.detections:
            self.global_detections_list[detection.name] = detection
    
    def callback_aruco(self, aruco_data):
        for detection in aruco_data.detections:
            self.global_detections_list[detection.name] = detection

    def callback_yellow(self, red_data):
        for detection in red_data.detections:
            if detection.id < 10:
                self.global_detections_list[detection.name] = detection
    
    def getOrganizedList(self, empty):
        rospy.loginfo("TOP CUBOS 2023:")
        
        def sortingKey(val):
            #LEMBRAR QUE ESTAMOS UTILIZANDO O COLOR OPTICAL FRAME!
            # abs(lateral)/distancia
            score = (abs(val.pose.position.x)+ 1)/(val.pose.position.z)
            rospy.loginfo("{} > score > {}".format(val.name, score))
            return score
        organized_list = sorted(self.global_detections_list.values(), key = sortingKey, reverse=True)
        detections = DetectionsOrganizedResponse()
        detections.cubes.detections = organized_list
        return detections

    def srvCallback(self, empty):
        self.global_detections_list = {}
        rospy.loginfo("*detecções limpas")
        return EmptyResponse()
        
def main(args):
    rospy.init_node('cube_organizer', anonymous=False)
    ic = cube_finder()
    rospy.loginfo("node started")
    rospy.Rate(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)