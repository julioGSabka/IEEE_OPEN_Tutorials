#!/usr/bin/env python3.8
from __future__ import print_function
from gc import callbacks

from requests import delete

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
from std_srvs.srv import SetBool, SetBoolResponse

class cube_finder:
    def __init__(self):
        self.bridge = CvBridge()
        self.transformBroadcaster = tf.TransformBroadcaster()
        #constantes de processamento de imagem
        self.kernelE = cv2.getStructuringElement(cv2.MORPH_RECT, (15,15))
        self.kernelD = cv2.getStructuringElement(cv2.MORPH_RECT, (15,15))
        self.lowerColor = np.array([0, 0, 0])
        self.upperColor = np.array([255, 255, 255])
        #self.srv = Server(ColorsConfig, self.conf_callback)
        #modelo da câmera
        self.cameraModel = PinholeCameraModel()
        
        #criar um sincronizador de tempo junto aos subrcribers, para suportar mais de uma inscrição por callback

        self.image_sub = message_filters.Subscriber("/d400/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/d400/aligned_depth_to_color/image_raw", Image)
        self.camera_info_sub = message_filters.Subscriber('/d400/color/camera_info', CameraInfo)

        self.frame_image_pub = rospy.Publisher("/d400_color_detector/detection_image", Image, queue_size=2)
        self.marker_pub = rospy.Publisher("/d400_color_detector/detection_marker", Marker, queue_size=10)

        #self.srv = rospy.Service("/enable_detection", SetBool, self.srvCallback)

        self.detections_pub = rospy.Publisher("/d400_color_detection", DetectionArray, queue_size=10)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.camera_info_sub], 2, 0.05)
        self.ts.registerCallback(self.callback)

    def conf_callback(self, config, level):
        self.lowerColor = np.array([config["hmin"], config["smin"], config["vmin"]])
        self.upperColor = np.array([config["hmax"], config["smax"], config["vmax"]])
        return config

    def callback(self,image_data, depth_data, camera_info_data):
        #tenta pegar a imagem do imgmsg
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
        except CvBridgeError as e:
            rospy.loginfo(e)
        self.cameraModel.fromCameraInfo(camera_info_data)
        frame = cv_image.copy()

        #borra a imagem para reduzir ruido
        blur = cv2.GaussianBlur(frame,(5,5),0)
        
        #transforma a imagem de RGB para HSV
        hsvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lowerColor = np.array([95, 100, 50])
        upperColor = np.array([125, 255,255])
    
        #definir os intervalos de cores que vao aparecer na imagem final
        mask = cv2.inRange(hsvImage, lowerColor, upperColor)

        #operações de Erode e Dilate para remover pixels perdidos
        mask = cv2.erode(mask, self.kernelE)
        mask = cv2.dilate(mask, self.kernelD)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        
        #encontra pontos que circundam regiões conexas (contour)
        edg = cv2.Canny(mask, 15, 15*3)
        contours, hierarchy = cv2.findContours(edg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        #se existir contornos
        if contours:
            detections = DetectionArray()
            #para cada grupo de pixels branco
            for id, cnt in enumerate(contours):
                xRect, yRect, wRect, hRect = cv2.boundingRect(cnt)
                cv2.putText(frame, str(id), (int(xRect+wRect/2), int(yRect+hRect*1.2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
                marker_x, marker_y, marker_z = self.pixel_to_coordinate(depth_image, int(xRect+wRect/2), int(yRect+hRect/2))
             #   if marker_z !=0 and marker_z < 0.4 and marker_x < 1.5:
              #      cube = Marker()
            #        cube.header.frame_id = "d400_color_optical_frame"
                #    cube.pose.position.x = marker_x
            #        cube.pose.position.y = marker_y
               #     cube.pose.position.z = marker_z + 0.03
                #    cube.color.a = 0.5
              #      cube.color.r = 1
           #         cube.scale.x = 0.06
              #      cube.scale.y = 0.06
               #     cube.scale.z = 0.06
            #        cube.id = int(id)
               #     cube.lifetime.from_sec(1/30)
             #       cube.type = cube.CUBE
              #      cube.pose.orientation.w=1
              #      self.marker_pub.publish(cube)

              #      detect = Detection()
             #       detect.id = int(id)
           #         detect.pose = cube.pose
             #       detect.header = cube.header
            #        detect.name = "d400_colored_cube_{}".format(id)
             #       detections.header.stamp = rospy.Time.now()
          #          detections.detections.append(detect)
             #       #if detect.pose.position.x > 0.66: Julio viajando

             #       self.transformBroadcaster.sendTransform(
           #             (cube.pose.position.x, cube.pose.position.y, cube.pose.position.z),
               #         (cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w),
             #           rospy.Time.now(), "colored_cube_{}".format(id), "{}_color_optical_frame".format("d400"))

            #self.detections_pub.publish(detections)
        
        
        #self.frame_image_pub.publish(self.bridge.cv2_to_imgmsg(mask))
        
    """
        Este código utiiza a matriz da camera para calcular o ponto xyz de uma depth cloud.
        Inspirado e retirado de:
        https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L91
        mesma técnica que é utilizada do rviz (eu acho)

        Inverse Projection Transformation
        https://towardsdatascience.com/inverse-projection-transformation-c866ccedef1c
    """
    def pixel_to_coordinate(self, depth, x, y):
        print(x, "", y)
        #extrair dados do modelo da câmera
        inv_fx = 1/self.cameraModel.fx()
        inv_fy = 1/self.cameraModel.fy()
        cx = self.cameraModel.cx()
        cy = self.cameraModel.cy()
        print("AAAAAAAAAAAAAAAAAA")
        print(self.cameraModel.fx())
        print(self.cameraModel.fy())
        print(cx)
        print(cy)
        print("AAAAAAAAAAAAAAAAAA")
        #extrair a profundidade do respectivo pixel e converter para metros
        z_val = depth.item(y, x) #i, j onde x=j e y=i
        point_z_metric = z_val * 0.001
        
        
        #extrapolar o raio da câmera
        point_x_metric = (x - cx) * point_z_metric * inv_fx
        point_y_metric = (y - cy) * point_z_metric * inv_fy
        print(point_x_metric)
        print(point_y_metric)
        print(point_z_metric)
        return point_x_metric, point_y_metric, point_z_metric

def main(args):
    rospy.init_node('colored_cube_finder', anonymous=True)
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