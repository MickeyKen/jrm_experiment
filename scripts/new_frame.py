#!/usr/bin/env python
import sys
import rospy
import tf
from std_msgs.msg import Float64, Int16
import time
from ubiquitous_display_pantilt.msg import Pantilt
from ubiquitous_display_msgs.srv import *
from dynamixel_controllers.srv import SetSpeed

class Control():
    def __init__(self):

        self.pantilt_radian_pub = rospy.Publisher('pantilt_radian_msg', Pantilt, queue_size=10)

        rospy.init_node('new_exp_control', anonymous=True)

        pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
        pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

        self.pt_sv_cl = rospy.ServiceProxy("/pantilt_radian_server", PantiltCommand)
        self.fin_pub = rospy.Publisher("/finish_pantilt", Int16, queue_size=10)

        float_pan = Float64()
        float_tilt = Float64()
        float_pan = -1.57

        exp_num = 0
        exp_pos = 1
        current_pos = 1
        rospy.set_param("/exp_num",exp_num)
        rospy.set_param("/exp_pos",exp_pos)

        self.make_pantilt(0.5,0.5,-1.57,0.0)

        while True:
            a = input("Input exp_num: >>")
            current_pos = rospy.get_param("exp_pos")
            rospy.set_param("/exp_miki_img/switch", 0)

            set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
            set_tilt_speed(0.3)

            pub_tilt.publish(float_tilt)

            time.sleep(3)

            if a == 1 or a == 2 or a == 3 or a == 4 or a == 91:
                rospy.set_param("/exp_num",a)

                if (current_pos == 1):
                    pass
                elif (current_pos == 2):
                    self.youbot_service("x", -200, 1000)
                    time.sleep(1)
                    self.youbot_service("y", -200, 2500)

                elif (current_pos == 3):
                    self.youbot_service("x", -200, 6000)
                    time.sleep(1)
                    self.youbot_service("y", -200, 2500)

                else:
                    pass

                rospy.set_param("/exp_pos",1)

                if a == 1:
                    self.make_pantilt(0.5,0.5,-1.57,0.5)
                    time.sleep(3)
                    self.fin_make()
                else:
                    self.make_pantilt(0.5,0.5,-1.57,1.1)
                    self.fin_make()

            elif a == 5 or a == 6 or a == 7 or a == 8 or a == 92:
                rospy.set_param("/exp_num",a)

                if (current_pos == 1):
                    self.youbot_service("y", 200, 2500)
                    time.sleep(1)
                    self.youbot_service("x", 200, 1000)

                elif (current_pos == 2):
                    pass

                elif (current_pos == 3):
                    self.youbot_service("x", -200, 5000)
                    pass

                else:
                    pass
                rospy.set_param("/exp_pos",2)

                if a == 5:
                    self.make_pantilt(0.5,0.5,-2.35,0.5)
                    time.sleep(3)
                    self.fin_make()
                else:
                    self.make_pantilt(0.5,0.5,-2.35,1.1)
                    self.fin_make()


            elif a == 9 or a == 10 or a == 11 or a ==12 or a == 93:
                rospy.set_param("/exp_num", a)

                if (current_pos == 1):
                    self.youbot_service("y", 200, 2500)
                    time.sleep(1)
                    self.youbot_service("x", 200, 6000)

                elif (current_pos == 2):
                    self.youbot_service("x", 200, 5000)
                    pass

                elif (current_pos == 3):
                    pass

                else:
                    pass
                rospy.set_param("/exp_pos",3)

                if a == 9:
                    self.make_pantilt(0.5,0.5,-3.91,0.5)
                    time.sleep(3)
                    self.fin_make()
                else:
                    self.make_pantilt(0.5,0.5,-3.91,1.1)
                    self.fin_make()

            elif a == 0:
                rospy.set_param("/exp_miki_img/switch", 0)
                rospy.set_param("/black_img/switch", 0)
                sys.exit()

            else:
                pass

    def make_pantilt(self, pan_speed, tilt_speed, pan_degree, tilt_degree):
        req = PantiltCommandRequest()
        req.pan_speed.data = pan_speed
        req.tilt_speed.data = tilt_speed
        req.pan_degree.data = pan_degree
        req.tilt_degree.data = tilt_degree
        res = self.pt_sv_cl(req)

    def fin_make(self):
        self.fin_pub.publish(1)


    def youbot_service(self, direction, speed, distance):
        rospy.wait_for_service('/ubiquitous_display/youbot')
        try:
            req = YoubotCommandRequest()
            call_youbot = rospy.ServiceProxy('/ubiquitous_display/youbot', YoubotCommand)
            req.direction.data = direction
            req.speed.data = speed
            req.distance.data = distance
            resp1 = call_youbot(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e




if __name__ == '__main__':
    Control()
