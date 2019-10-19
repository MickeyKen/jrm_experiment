#!/usr/bin/env python
import sys
import rospy
import tf
from std_msgs.msg import Float64
import time
from ubiquitous_display_pantilt.msg import Pantilt
from geometry_msgs.msg import Twist, Point, Quaternion
import PyKDL
from math import radians, copysign, sqrt, pow, pi



class Control():
    def __init__(self):

        pantilt_radian_pub = rospy.Publisher('pantilt_radian_msg', Pantilt, queue_size=10)
        pantilt_message = Pantilt()

        rospy.init_node('exp_control', anonymous=True)

        self.pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()

        self.vel_msg = Twist()
        self.position = Point()
        self.rotation = Quaternion()

        self.odom_frame = '/odom'
        self.base_frame = '/base_footprint'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between /odom and /ud_base_footprint")
            rospy.signal_shutdown("tf Exception")
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        rate = 20
        self.r = rospy.Rate(rate)

        ##### set initial tilt and pan radian #####
        position_1_tilt = 0.0
        position_1_pan = 0.0

        exp_num = 0
        exp_pos = 1
        current_pos = 1

        pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
        pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

        float_pan = Float64()
        float_tilt = Float64()

        rospy.set_param("/exp_num",exp_num)
        rospy.set_param("/exp_pos",exp_pos)

        pantilt_message.speed.x = 0.5
        pantilt_message.speed.y = 0.5
        pantilt_message.speed.z = 0.0
        pantilt_message.position.x = -1.57
        pantilt_message.position.y = 1.1
        pantilt_message.position.z = 0.0
        pantilt_radian_pub.publish(pantilt_message)

        while True:
            a = input("Input exp_num: >>")
            # print a
            current_pos = rospy.get_param("exp_pos")
            rospy.set_param("/exp_miki_img/switch", 0)

            pub_pan.publish(float_pan)
            pub_tilt.publish(float_tilt)
            time.sleep(5)

            if a == 1 or a == 2 or a == 3 or a == 4:
                rospy.set_param("/exp_num",a)

                if (current_pos == 1):
                    pass
                elif (current_pos == 2):
                    self.move(current_pos, 1, -1.0, -2.5)

                elif (current_pos == 3):
                    self.move(current_pos, 1, -6.0, -2.5)

                else:
                    pass

                rospy.set_param("/exp_pos",1)

                pantilt_message.speed.x = 0.5
                pantilt_message.speed.y = 0.5
                pantilt_message.speed.z = 0.0
                pantilt_message.position.x = -1.57
                pantilt_message.position.y = 1.1
                pantilt_message.position.z = 0.0
                pantilt_radian_pub.publish(pantilt_message)

            elif a == 5 or a == 6 or a == 7 or a == 8:
                rospy.set_param("/exp_num",a)

                if (current_pos == 1):
                    self.move(current_pos, 2, 1.0, 2.5)

                elif (current_pos == 2):
                    pass

                elif (current_pos == 3):
                    self.move(current_pos, 2, -5.0, 0.0)

                else:
                    pass
                rospy.set_param("/exp_pos",2)

                pantilt_message.speed.x = 0.5
                pantilt_message.speed.y = 0.5
                pantilt_message.speed.z = 0.0
                pantilt_message.position.x = -2.35
                pantilt_message.position.y = 1.1
                pantilt_message.position.z = 0.0
                pantilt_radian_pub.publish(pantilt_message)


            elif a == 9 or a == 10 or a == 11 or a ==12:
                rospy.set_param("/exp_num", a)

                if (current_pos == 1):
                    self.move(current_pos, 3, 6.0, 2.5)

                elif (current_pos == 2):
                    self.move(current_pos, 3, 5.0, 0.0)

                elif (current_pos == 3):
                    pass

                else:
                    pass
                rospy.set_param("/exp_pos",3)

                pantilt_message.speed.x = 0.5
                pantilt_message.speed.y = 0.5
                pantilt_message.speed.z = 0.0
                pantilt_message.position.x = -3.91
                pantilt_message.position.y = 1.1
                pantilt_message.position.z = 0.0
                pantilt_radian_pub.publish(pantilt_message)


            elif a == 0:
                rospy.set_param("/exp_miki_img/switch", 0)
                rospy.set_param("/black_img/switch", 0)
                sys.exit()
            else:
                pass

    def quat_to_angle(quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def get_odom(self):

        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), Quaternion(*rot))

    def move(self, curren_pos, target_pos, x, y):
        # print x, y
        current_x_distance = 0.0
        current_y_distance = 0.0


        ### get initial position
        (position, rotation) = self.get_odom()
        initial_x = position.x
        initial_y = position.y

        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0

        if curren_pos < target_pos:
            if (y == 0.0):
                pass
            else:
                while( current_y_distance < abs(y)):
                    (position, rotation) = self.get_odom()
                    current_y_distance = sqrt(pow((position.y - initial_y), 2))
                    if (self.vel_msg.linear.y <= 0.3):
                        self.vel_msg.linear.y += 0.03
                    self.pub_vel.publish(self.vel_msg)
                    self.r.sleep()

                self.vel_msg.linear.y = 0.0
                self.pub_vel.publish(self.vel_msg)

                time.sleep(2)

            if (x == 0.0):
                pass
            else:
                while( current_x_distance < abs(x)):
                    (position, rotation) = self.get_odom()
                    current_x_distance = sqrt(pow((position.x - initial_x), 2))
                    if (self.vel_msg.linear.x <= 0.3):
                        self.vel_msg.linear.x += 0.03
                    self.pub_vel.publish(self.vel_msg)
                    self.r.sleep()

                self.vel_msg.linear.x = 0.0
                self.pub_vel.publish(self.vel_msg)
        else:

            if (x == 0.0):
                pass
            else:
                while( current_x_distance < abs(x)):
                    (position, rotation) = self.get_odom()
                    current_x_distance = sqrt(pow((position.x - initial_x), 2))
                    if (self.vel_msg.linear.x >= -0.3):
                        self.vel_msg.linear.x -= 0.03
                    self.pub_vel.publish(self.vel_msg)
                    self.r.sleep()

                self.vel_msg.linear.x = 0.0
                self.pub_vel.publish(self.vel_msg)

            time.sleep(2)
            if (y == 0.0):
                pass
            else:
                while( current_y_distance < abs(y)):
                    (position, rotation) = self.get_odom()
                    current_y_distance = sqrt(pow((position.y - initial_y), 2))
                    if (self.vel_msg.linear.y >= -0.3):
                        self.vel_msg.linear.y -= 0.03
                    self.pub_vel.publish(self.vel_msg)
                    self.r.sleep()

                self.vel_msg.linear.y = 0.0
                self.pub_vel.publish(self.vel_msg)

if __name__ == '__main__':
    Control()
