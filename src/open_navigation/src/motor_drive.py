#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from strategy.msg import FourMotorsOiBB

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

class wheel_driver:
    
    def __init__(self):
        self.WHEEL_RADIUS = 0.48
        self.WHEEL_SEPARATION_WIDTH = 0.17
        self.WHEEL_SEPARATION_LENGTH = 0.25
        self.update_sub = rospy.Subscriber("/cmd_vel",Twist,self.callback)
        self.vel_pub = rospy.Publisher("/four_motor_vel", FourMotorsOiBB, queue_size=1)

    def callback(self,data):
        angular = data.angular
        linear = data.linear
        wheel_front_left = (1/self.WHEEL_RADIUS) * ((linear.x - linear.y) - (self.WHEEL_SEPARATION_WIDTH + self.WHEEL_SEPARATION_LENGTH)*angular.z)
        wheel_front_right = (1/self.WHEEL_RADIUS) * ((linear.x + linear.y) + (self.WHEEL_SEPARATION_WIDTH + self.WHEEL_SEPARATION_LENGTH)*angular.z)
        wheel_rear_left = (1/self.WHEEL_RADIUS) * ((linear.x + linear.y) - (self.WHEEL_SEPARATION_WIDTH + self.WHEEL_SEPARATION_LENGTH)*angular.z)
        wheel_rear_right = (1/self.WHEEL_RADIUS) * ((linear.x - linear.y) + (self.WHEEL_SEPARATION_WIDTH + self.WHEEL_SEPARATION_LENGTH)*angular.z)
        
        self.m3 = clamp((wheel_front_left*0.5) + 0, -1, 1)
        self.m2 = clamp((-wheel_front_right*0.5) + 0, -1, 1)
        self.m4 = clamp((wheel_rear_left*0.5) + 0, -1, 1)
        self.m1 = clamp((-wheel_rear_right*0.5) + 0, -1, 1)

        vel = FourMotorsOiBB()
        vel.m1 = self.m1
        vel.m2 = self.m2
        vel.m3 = self.m3
        vel.m4 = self.m4

        self.vel_pub.publish(vel)

        #print("m1:{} m2:{} m3:{} m4:{}".format(self.m1, self.m2, self.m3, self.m4))

def main(args):
  rospy.init_node('base_finder', anonymous=False)
  ic = wheel_driver()
  rospy.loginfo("base_finder node started")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")

if __name__ == '__main__':
  main(sys.argv)