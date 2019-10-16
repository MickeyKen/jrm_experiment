#!/usr/bin/env python
import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray
import rospy
import time
import sys

def black_img():

    rospy.init_node('black_img', anonymous=True)

    rospy.set_param("black_img/switch", 1)

    black_image = np.zeros((678,1024,3),np.uint8)

    cv2.namedWindow('window')
    cv2.namedWindow('screen', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('screen', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow('screen', black_image)
    cv2.waitKey(1000)
    while True:
      if rospy.get_param("black_img/switch") == 0:
        cv2.destroyAllWindows()
        sys.exit()
      else:
        pass



if __name__ == '__main__':
    black_img()
