#!/usr/bin/env python3

import rospy
import sys, time
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
from opencv_open.srv import DetectionsOrganized, DetectionsOrganizedResponse
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import Float32, UInt16, Int16
from transitions import Machine
from tf2_ros import TransformBroadcaster, TransformListener, TransformStamped, Buffer
import tf
import math 

class CustomMachine:

    def __init__(self):

        self.pos_x = None
        self.pos_y = None
        self.orient = None
        self.first_update_received = False
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        
        # Conexão com Serviços Exteriores
        self.dynamic_obstacle = rospy.ServiceProxy("/arena_limits", SetBool, persistent=False)
        self.detector_enabled = rospy.ServiceProxy("/enable_detection", SetBool, persistent=False)
        self.list_cleaner = rospy.ServiceProxy("/list_cleanner", Empty, persistent=False)
        self.detect_organizer = rospy.ServiceProxy("/get_all_detections_organized", DetectionsOrganized, persistent=False)

        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        trans, rot = self.listener.lookupTransform('/map', '/base_link', rospy.Time())
        self.pos_x = trans[0]
        self.pos_y = trans[1]
        self.orient = tf.transformations.euler_from_quaternion(rot)[2]
        
    def limit(self, val, min, max):
        if val>max:
            return max
        if val < min:
            return min
        return val
    def alignCube(self, desiredX, desiredY, cube):
        vel_move = Twist()
        trans, rot = self.listener.lookupTransform('/d400_color_optical_frame', "/{}".format(cube),  rospy.Time())
        print(trans)
        
        while((abs(trans[0] - desiredX) > 0.03 or abs(trans[2] - desiredY) > 0.03) and (not rospy.is_shutdown())):
            try:
                self.listener.waitForTransform('/d400_color_optical_frame', "/{}".format(cube), rospy.Time(), rospy.Duration(1.0))
                trans, rot = self.listener.lookupTransform('/d400_color_optical_frame', "/{}".format(cube),  rospy.Time())
                print(trans)
                vel_move.linear.y = self.limit(-(trans[0] - desiredX) * 2, -.2, .2)
                vel_move.linear.x = self.limit((trans[2] - desiredY) * 2, -.2, .2)

                print(abs(trans[0] - desiredX))
                self.vel_pub.publish(vel_move)
            except:
                print("N foi possivel TEeFar")
            rospy.sleep(1/50)
        vel_move.linear.x = 0
        vel_move.linear.y = 0
        self.vel_pub.publish(vel_move)

    def main(self):

        self.state = 0
        rate = rospy.Rate(1)

        rospy.loginfo("main started")

        self.alignCube(0.05, 0.2, "aruco_cube_3")
             
def main(args):
    rospy.init_node('strategy', anonymous=False)

    machine = CustomMachine()

    rospy.loginfo("node started")

    machine.main()

if __name__ == '__main__':
    main(sys.argv)
    

