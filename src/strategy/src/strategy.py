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
        self.lift_pos_pub = rospy.Publisher("/lift_pos", Int16, queue_size=1)
        self.pulso_pub = rospy.Publisher("/pulso_pos", UInt16, queue_size=1)
        self.garra_pub = rospy.Publisher("/garra_pos", UInt16, queue_size=1)
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
        
    def updateBotPos(self):
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
        trans, rot = self.listener.lookupTransform('/map', '/base_link', rospy.Time())
        self.pos_x = trans[0]
        self.pos_y = trans[1]
        self.orient = tf.transformations.euler_from_quaternion(rot)[2]

    def lift_pos(self, pos):
        msgL = Int16()
        msgL.data = pos
        self.lift_pos_pub.publish(msgL)

    def pulso_pos(self, pos):
        msgP = UInt16()
        msgP.data = pos
        self.pulso_pub.publish(msgP)   

    def garra_pos(self, pos):
        msgG = UInt16()
        msgG.data = pos
        self.garra_pub.publish(msgG)   

    def alignCube(self, desiredX, desiredY):
        vel_move = Twist()
        trans, rot = self.listener.lookupTransform('/d400_color_optical_frame', "/{}".format(self.melhor_cubo.name),  rospy.Time())
        print(trans)
        
        while((abs(trans[0] - desiredX) > 0.03 or abs(trans[2] - desiredY) > 0.03) and (not rospy.is_shutdown())):
            try:
                self.listener.waitForTransform('/d400_color_optical_frame', "/{}".format(self.melhor_cubo.name), rospy.Time(), rospy.Duration(1.0))
                trans, rot = self.listener.lookupTransform('/d400_color_optical_frame', "/{}".format(self.melhor_cubo.name),  rospy.Time())
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
 
    def colect_cube(self):

        print("Colect Cube")
        self.list_cleaner()
        rospy.sleep(rospy.Duration(3))
        organized = self.detect_organizer()
        rospy.loginfo(organized)

        #Desativar Obstaculo Dinamico
        self.dynamic_obstacle(False)
        print("DynObs: False")
        if(len(organized.cubes.detections) > 0):
            i = 0
            trans = None
            #Go to cube coordinate
            while trans == None and i < len(organized.cubes.detections):
                self.melhor_cubo = organized.cubes.detections[i]
                print(self.melhor_cubo.name)
                rospy.sleep(rospy.Duration(1))
                try:
                    # obtem a coordenada do cubo referente ao mapa
                    self.listener.waitForTransform('/map', "/{}".format(self.melhor_cubo.name), rospy.Time(), rospy.Duration(1.0))
                    trans, rot = self.listener.lookupTransform('/map', "/{}".format(self.melhor_cubo.name),  rospy.Time())
                except:
                    rospy.logwarn("No cubes found!! (lascou)")
                    trans = None
                    i = i + 1
            if(trans == None):
                return 0
            print("Trans:", trans)

            self.alignCube(0.05, 0.2)
            rospy.sleep(rospy.Duration(1))
            self.dumb_move(0.08, 1.5)
            
            #Move the lift
            self.lift_pos(0)

            self.garra_pos(0)
            #Get Cube
            self.pulso_pos(170)
            rospy.sleep(rospy.Duration(1))
            self.updateBotPos()
            if(math.degrees(self.pos_y>0)):
                sign = 1 #1 = lado das prateleiras -1 lado das cores
            else:
                sign = -1 #1 = lado das prateleiras -1 lado das cores
            #vai ate ele
            print(sign)
            self.move_to_coordinate(trans[0]-0.02*sign, (trans[1] + 0.15 * sign), -90*sign)
            #Forward
            self.dumb_move(0.08, 1.5)
            
            self.garra_pos(40)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(0)
            
            #Backward
            self.dumb_move(-0.08, 1.5)

            print("Cubo Coletado")
            return 1
        else:
            return 0

    def deposit_cube(self):

        print("Deposit Cube")
        #Ativar Obstaculo Dinamico
        self.dynamic_obstacle(True)
        rospy.sleep(rospy.Duration(1))
            
        #Go to shelf coordinate and Move the lift up
        self.switch(self.melhor_cubo.name)
        
        #Desativar Obstaculo Dinamico
        self.dynamic_obstacle(False)
        rospy.sleep(rospy.Duration(1))

        #Pulso Reto
        self.pulso_pos(120)
        rospy.sleep(rospy.Duration(1))

        #Forward
        self.dumb_move(0.08, 2)

        #Open Claw
        self.garra_pos(20)
        rospy.sleep(rospy.Duration(1))

        #Backward
        self.dumb_move(-0.07, 1.3)

        print("Cubo Depositado")
    
        return 0
    
    def switch(self, type):
        if type == "aruco_cube_1":
            self.move_to_coordinate(-0.98, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(0)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(160)
        elif type == "aruco_cube_2":
            self.move_to_coordinate(-0.64, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(0)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(160)
        elif type == "aruco_cube_3":
            self.move_to_coordinate(-0.32, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(0)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(160)
        elif type == "aruco_cube_4":
            self.move_to_coordinate(-0.98, 0.79, 90)
            rospy.sleep(rospy.Duration(1)) 
            self.lift_pos(0)    
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(120)
        elif type == "aruco_cube_5":
            self.move_to_coordinate(-0.64, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(0)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(120)
        elif type == "aruco_cube_6":
            self.move_to_coordinate(-0.32, 0.79, 90)
            rospy.sleep(rospy.Duration(1)) 
            self.lift_pos(0) 
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(120)
        elif type == "aruco_cube_7":
            self.move_to_coordinate(-0.98, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(-1280)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(120)
        elif type == "aruco_cube_8":
            self.move_to_coordinate(-0.64, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(-1280)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(120) 
        elif type == "aruco_cube_9":
            self.move_to_coordinate(-0.32, 0.79, 90)
            rospy.sleep(rospy.Duration(1))
            self.lift_pos(-1280)
            rospy.sleep(rospy.Duration(1))
            self.pulso_pos(120)
        elif type == "Red":
            self.move_to_coordinate(0.0, -0.66, 90)
        elif type == "Green":
            self.move_to_coordinate(-0.98, -0.66, 90)
        elif type == "Blue":
            self.move_to_coordinate(-0.64, -0.66, 90)
        elif type == "Yellow":
            self.move_to_coordinate(-0.32, -0.66, 90)
          

    def move_to_coordinate(self, x , y , orientation):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.y = y
        goal.pose.position.x = x
        goal.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,math.radians(orientation))) 
        self.goal_pub.publish(goal)
        rospy.sleep(0.5)
        self.goal_pub.publish(goal)
                
        
        #modo certo de achara GOAL
        while(rospy.wait_for_message("/move_base/result", MoveBaseActionResult).status.status !=3 and not rospy.is_shutdown()):
            pass
        
        """
        self.updateBotPos()
        while(math.sqrt(((self.pos_x - goal.pose.position.x) **2) * ((self.pos_y - goal.pose.position.y)**2)) > 0.1):
            self.updateBotPos()
            rospy.sleep(0.1)
        rospy.sleep(5)
        """
    def alignCube(self, desiredX, desiredY):
        vel_move = Twist()
        while(abs(vel_move.linear.x) < 0.05 and abs(vel_move.linear.y) < 0.05 and not rospy.is_shutdown()):
            trans, rot = self.listener.lookupTransform('/map', "/{}".format(self.melhor_cubo.name),  rospy.Time())
            vel_move.linear.x = (trans[0] - desiredX) * 1
            vel_move.linear.y = (trans[1] - desiredY) * 1
            self.vel_pub.publish(vel_move)
        vel_move.linear.x = 0
        vel_move.linear.y = 0
        self.vel_pub.publish(vel_move)

    def dumb_move(self, vel, time):
        vel_back = Twist()
        vel_back.linear.x = vel

        rospy.loginfo("MovBurro")
        self.vel_pub.publish(vel_back)
        rospy.sleep(rospy.Duration(time))
        self.vel_pub.publish(Twist())
        rospy.sleep(rospy.Duration(1.5))
    
    def gotoQuad(self, quadrant):
        if quadrant == 3: #quadrante superior esquerdo
            self.move_to_coordinate(-0.64, 0.48, -90)
        if quadrant == 2: #quadrante inferior esquerdo
            self.move_to_coordinate(-0.64, -0.48, 90)
        if quadrant == 1: #quadrante inferior direito
            self.move_to_coordinate(0.64, -0.48, 90)
        if quadrant == 0: #quadrante superior direito
            self.move_to_coordinate(0.64, 0.48, -90)

    def main(self):

        self.state = 0
        rate = rospy.Rate(1)
        self.dynamic_obstacle(True)
        rospy.sleep(1)
        self.pulso_pub.publish(0)

        rospy.loginfo("main started")
        self.lift_pos(-1300)
        rospy.sleep(rospy.Duration(2))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        #rospy.loginfo("a")


        currQuad = 0

        if (self.pos_y > 0):
            if(self.pos_x>0):
                currQuad = 0
            else:
                currQuad = 3
        else:
            if(self.pos_x>0):
                currQuad = 1
            else:
                currQuad = 2

        while(not rospy.is_shutdown()):
            if (self.state == 0):
                self.dynamic_obstacle(True)
                rospy.sleep(rospy.Duration(1))
                rospy.logwarn("indo para: " + str(currQuad))
                self.gotoQuad(currQuad)
                rospy.logwarn("CHEGUEI!!!!!")
                self.state = self.colect_cube()
                if(self.state == 0):
                    currQuad = currQuad + 1
            if currQuad > 3:
                currQuad = 0 
            if (self.state == 1):
                self.state = self.deposit_cube()
            rate.sleep()
             
def main(args):
    rospy.init_node('strategy', anonymous=False)

    machine = CustomMachine()

    rospy.loginfo("node started")

    machine.main()

if __name__ == '__main__':
    main(sys.argv)
    

