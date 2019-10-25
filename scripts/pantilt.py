#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64,Int16
import time
from dynamixel_controllers.srv import SetSpeed

def callback(data):
    time.sleep(2)

    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
    exp_num = rospy.get_param("/exp_num")

    tilt_message = Float64()

    tilt_speed= 0.3

    # slow
    if (exp_num == 2 or exp_num  == 6 or exp_num == 10 or exp_num == 91 or exp_num == 92 or exp_num == 93):
        if exp_num == 2:
            tilt_speed= 0.06
        elif exp_num == 6:
            tilt_speed= 0.06
        elif exp_num == 10:
            tilt_speed= 0.06
        else:
            tilt_speed = 0.06

    # mediaum
    elif (exp_num == 3 or exp_num == 7 or exp_num == 11):
        if exp_num == 3:
            tilt_speed= 1.26
        elif exp_num == 7:
            tilt_speed= 0.72
        elif exp_num == 11:
            tilt_speed= 0.54
        else:
            tilt_speed = tilt_speed

    # # fast
    elif (exp_num == 4 or exp_num  == 8 or exp_num == 12):
	return 
    #     if exp_num == 4:
    #         tilt_speed= 2.52
    #     elif exp_num == 8:
    #         tilt_speed= 1.44
    #     elif exp_num == 12:
    #         tilt_speed= 1.08
    #     else:
    #         tilt_speed = tilt_speed
    # else:
    #     pass

    set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
    set_tilt_speed(tilt_speed)

    # angular
    tilt_message = 0.5
    pub_tilt.publish(tilt_message)


def experiment_pantilt():

    rospy.init_node('experiment_for_pantilt', anonymous=True)

    rospy.Subscriber("finish_pantilt", Int16, callback)
    rospy.spin()

if __name__ == '__main__':
    experiment_pantilt()
