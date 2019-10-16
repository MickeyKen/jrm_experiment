#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64,Int16
import time
from dynamixel_controllers.srv import SetSpeed

def callback(data):
    # print data.data
    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
    exp_num = rospy.get_param("/exp_num")

    tilt_message = Float64()

    time.sleep(1)

    tilt_speed= 0.3


    if (exp_num == 1 or exp_num == 5 or exp_num == 9):
        tilt_speed= 0.5

    # slow
    elif (exp_num == 2 or exp_num  == 6 or exp_num == 10):
        if exp_num == 2:
            tilt_speed= 0.06
        elif exp_num == 6:
            tilt_speed= 0.06
        else:
            tilt_speed= 0.06

    # mediaum
    elif (exp_num == 3 or exp_num == 7 or exp_num == 11):
        if exp_num == 3:
            tilt_speed= 1.26
        elif exp_num == 7:
            tilt_speed= 0.72
        else:
            tilt_speed= 0.54

    # fast
    elif (exp_num == 4 or exp_num  == 8 or exp_num == 12):
        if exp_num == 4:
            tilt_speed= 2.52
        elif exp_num == 8:
            tilt_speed= 1.44
        else:
            tilt_speed= 1.08
    else:
        pass

    set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
    set_tilt_speed(tilt_speed)

    tilt_message = 0.5

    pub_tilt.publish(tilt_message)

    time.sleep(3)

    tilt_speed = 0.5
    set_tilt_speed(tilt_speed)




def experiment_pantilt():

    rospy.init_node('experiment_for_pantilt', anonymous=True)

    rospy.Subscriber("finish_pantilt", Int16, callback)
    rospy.spin()

if __name__ == '__main__':
    experiment_pantilt()
