#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
import cv2

from std_msgs.msg import String, Float64, Bool, Int16
from sensor_msgs.msg import JointState
from ubiquitous_display_msgs.srv import PantiltCommand
from ubiquitous_display_msgs.srv import PantiltCommandResponse
from dynamixel_controllers.srv import SetSpeed

class Publishsers():

    def pan_make(self, speed, rad):
        self.set_pan_speed(speed)
        self.pan_pub.publish(rad)

    def tilt_make(self, speed, rad):
        self.set_tilt_speed(speed)
        self.tilt_pub.publish(rad)

class Server(Publishsers):
    def __init__(self):
        self.pan_rad = 0.0
        self.tilt_rad = 0.0

        self.pan_offset = 0.04
        self.tilt_offset = 0.04

        self.rate = rospy.Rate(100)

        self.tilt_speed = 0.0

        self.pan_pub = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
        self.set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
        self.set_pan_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)

        # Declaration Subscriber
        self.ptm_sub = rospy.Subscriber('/pantilt_joint_states', JointState , self.djs_callback)
        self.fin_sub = rospy.Subscriber('/finish_pantilt', Int16 , self.main_callback)

        self.black_image = np.zeros((768,1024,3),np.uint8)

    def djs_callback(self, msg):
        self.pan_rad = msg.position[0]
        self.tilt_rad = msg.position[1]


    def main_callback(self, msg):

        time.sleep(2)

        exp_num = rospy.get_param("/exp_num")

        tilt_degree = 0.5

        dcc_step = 0.0
        self.thread = 0.65

        if exp_num == 4:
            self.tilt_speed = 2.52
            dcc_step = 0.5

        elif exp_num == 8:
            self.tilt_speed = 1.44
            dcc_step = 0.25

        elif exp_num == 12:
            self.tilt_speed = 1.08
            dcc_step = 0.125

        else:
            self.tilt_speed = 0.3
            dcc_step = dcc_step

        self.tilt_make(self.tilt_speed, tilt_degree)

        while True:
            if self.tilt_rad > 0.5 - self.tilt_offset and self.tilt_rad < 0.5 + self.tilt_offset:
                break
            elif self.tilt_rad < self.thread:
                self.tilt_speed = 0.3
                self.tilt_make(self.tilt_speed, 0.5)
                break
            else:
                pass

            self.rate.sleep()
        cv2.namedWindow('em')
        cv2.namedWindow('em', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('em', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow('em', self.black_image)
        cv2.waitKey(1)
        time.sleep(0.2)
        cv2.destroyWindow("em")
        print self.tilt_speed


if __name__ == '__main__':
    rospy.init_node('pantilt_radian_server_for_fast')

    server = Server()

    rospy.spin()
