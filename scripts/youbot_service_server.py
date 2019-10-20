#!/usr/bin/env python

import rospy
import numpy as np
from math import radians, copysign, sqrt, pow, pi
import time
import PyKDL

import tf

from std_msgs.msg import String, Int64, Float64, Bool
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from ubiquitous_display_msgs.srv import YoubotCommand
from ubiquitous_display_msgs.srv import YoubotCommandResponse

class Publishsers():

    def cmd_make(self, x, y, z):
        cmd_msg = Twist()

        cmd_msg.linear.x = x
        cmd_msg.linear.y = y
        cmd_msg.angular.z = z

        self.cmd_pub.publish(cmd_msg)


class Server(Publishsers):
    def __init__(self):
        self.PI = 3.1415926535897
        self.MAX_LINEAR_VALUE = 0.8
        self.MIN_LINEAR_VALUE = -(0.8)
        self.MAX_ANGULAR_VALUE = 2.0
        self.MIN_ANGULAR_VALUE = -(2.0)
        self.current_linear = 0.0
        self.rate = rospy.Rate(100) # 100hz
        self.rate_time = 0.01
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except(tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # message for result topic
        self.result = String()

        # self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry , self.odom_callback)

        # Declaration Service Server
        self.server = rospy.Service("/ubiquitous_display/youbot", YoubotCommand, self.service_callback)

    def service_callback(self, req):
        result = Bool()
	    result = False

        order = str(req.direction.data)

        if order == "x":
            acc = req.acceleration.data
            acc = acc / 1000.0

            speed = req.speed.data
            speed = speed / 1000.0
            speed = self.constrain(speed, self.MIN_LINEAR_VALUE, self.MAX_LINEAR_VALUE)

            distance = req.distance.data
            distance = distance / 1000.0

            dec = req.deceleration.data
            dec = dec / 1000.0

            # current_distance = self.acceleration(acc, 0.0, speed)
            current_distance = 0.0
            result.data = self.linear(acc, speed, 0.0, distance, dec, current_distance)

        elif order == "y":
            acc = req.acceleration.data
            acc = acc / 1000.0

            speed = req.speed.data
            speed = speed / 1000.0
            speed = self.constrain(speed, self.MIN_LINEAR_VALUE, self.MAX_LINEAR_VALUE)

            distance = req.distance.data
            distance = distance / 1000.0

            dec = req.deceleration.data
            dec = dec / 1000.0

            current_distance = 0.0
            result.data = self.linear(acc, 0.0, speed, distance, dec, current_distance)

        elif order == "z":
            omega = req.omega.data
            angle = req.angle.data

            relative_angle = angle * 2 * self.PI / 360  # convert from degree to radian

            result.data = self.angular(omega, relative_angle)

        elif order == "s":
            dec = req.deceleration.data
            if dec:
                dec = dec / 1000.0
            else:
                dec = 0.5   #m/s
            result.data = self.stop(dec)

        else:
            print "else"

        return YoubotCommandResponse(result)

    def odom_callback(self, odom):

        self.current_linear_x = odom.twist.twist.linear.x
        self.current_linear_y = odom.twist.twist.linear.y
        self.current_angular = odom.twist.twist.angular.z
        # self.left_velocity = odom.twist.twist.linear.x + (odom.twist.twist.angular.z * self.HALF_WHEEL_SEPARATION)
        # self.right_velocity = odom.twist.twist.linear.x - (odom.twist.twist.angular.z * self.HALF_WHEEL_SEPARATION

    def acceleration(self, xacc, yacc, target_speed):
        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y

        if xacc == 0.0:
            while (abs(self.current_linear_y) < abs(target_speed)):
                xvel += xacc * self.rate_time
                yvel += yacc * self.rate_time
                self.cmd_make(xvel, yvel, 0.0)
                (position, rotation) = self.get_odom()
                current_distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
                self.rate.sleep()
        else:
            while (abs(self.current_linear_x) < abs(target_speed)):
                xvel += xacc * self.rate_time
                yvel += yacc * self.rate_time
                self.cmd_make(xvel, yvel, 0.0)
                (position, rotation) = self.get_odom()
                current_distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
                self.rate.sleep()
            # self.current_linear = (self.left_velocity + self.right_velocity) / 2
        return current_distance

    def linear(self, acc, xvel, yvel, distance, dec, c_distance):
        current_distance = 0.0

        (position, rotation) = self.get_odom()
        x_start = position.x
        y_start = position.y

        while( abs(current_distance) < abs(distance)):
            self.cmd_make(xvel, yvel, 0.0)
            self.rate.sleep()
            (position, rotation) = self.get_odom()
            current_distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2)) + c_distance

        self.cmd_make(0.0, 0.0, 0.0)
        return True

    def angular(self, omega, relative_angle):
        angular_tolerance = radians(2.5)

        (prev_position, prev_rotation) = self.get_odom()
        last_angle = prev_rotation
        turn_angle = 0.0

        self.cmd_make(0.0, 0.0, omega)

        while( abs(turn_angle + angular_tolerance) < abs(relative_angle)):

            self.rate.sleep()
            (position, rotation) = self.get_odom()
            delta_angle = self.normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation

        self.cmd_make(0.0, 0.0, 0.0)

        return True

    def stop(self, down_acc):

        self.cmd_make(0.0, 0.0, 0.0)
        return True

    def constrain(self, linear_value, MIN_VALUE, MAX_VALUE):
        if linear_value < MIN_VALUE:
            linear_value = MIN_VALUE
        elif linear_value > MAX_VALUE:
            linear_value = MAX_VALUE
        else:
            linear_value = linear_value

        return linear_value

    def get_distance(self, distance_x, distance_y):
        distance = sqrt(distance_x**2 + distance_y**2)
        return distance

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

if __name__ == '__main__':
    rospy.init_node('youbot_service_server')

    server = Server()

    rospy.spin()
