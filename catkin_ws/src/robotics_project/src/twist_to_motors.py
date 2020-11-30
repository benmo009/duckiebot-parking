#!/usr/bin/env python

#Duckibot Driver ROS subscriber in Python

import time
import threading
import numpy
import traceback
import sys
# Import these to control motor hats
from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

#####################TODO:

#import ROS python library and Twist type message 
from geometry_msgs.msg import Twist
import rospy


#-----------------------------------------------------------------
class DaguWheelsDriver:
    LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  
    RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  
    SPEED_TOLERANCE = 1.e-2       # speed tolerance level

    def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)
        self.verbose = verbose or debug
        self.debug = debug

        self._lock=threading.RLock()
        self._streaming=False
        self._ep=0
        # Set directions based on flip property
        self.left_sgn = 1.0
        if left_flip:
            self.left_sgn = -1.0
        self.right_sgn = 1.0
        if right_flip:
            self.right_sgn = -1.0
        # Set initial speeds to zero
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self.updatePWM()

        self.setupROS()

    def PWMvalue(self, v, minPWM, maxPWM):
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
        return min(pwm, maxPWM)

    def updatePWM(self):
        vl = self.leftSpeed*self.left_sgn
        vr = self.rightSpeed*self.right_sgn

        pwml = self.PWMvalue(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self.PWMvalue(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)

        if self.debug:
            #print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl, vr, pwml, pwmr)
            print "vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (vl, vr, pwml, pwmr)

        if fabs(vl) < self.SPEED_TOLERANCE:
            leftMotorMode = Adafruit_MotorHAT.RELEASE
        elif vl > 0:
            leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif vl < 0: 
            leftMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            rightMotorMode = Adafruit_MotorHAT.RELEASE
            pwmr = 0
        elif vr > 0:
            rightMotorMode = Adafruit_MotorHAT.FORWARD
        elif vr < 0: 
            rightMotorMode = Adafruit_MotorHAT.BACKWARD

        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(leftMotorMode)
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(rightMotorMode)

    def setWheelsSpeed(self, left, right):
        with self._lock:
            self.leftSpeed = left
            self.rightSpeed = right
            self.updatePWM()

    # Stop the streaming thread
    def StopStreaming(self):
        if (not self._streaming):
            raise Exception("Not streaming")
        self._streaming=False

    def Shutdown(self):
        with self._lock:
            self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
            self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
            del self.motorhat

#-----------------------------------------------------------------

    b = 0.1
    LINEAR_SCALING = 0.5/0.322
    ROTATION_SCALING = 2
    def fwdkin(self, twist):
        v = twist.linear.x * self.LINEAR_SCALING
        omega = twist.angular.z * self.ROTATION_SCALING

        vl = v-omega*self.b/2
        vr = v+omega*self.b/2
        if self.debug:
            print "fwdkin: vl = %5.3f, vr = %5.3f, v = %5.3f, omega = %5.3f" % (vl, vr, v, omega)
        return vl,vr

    LEFT_MAX_SPEED = 0.776
    RIGHT_MAX_SPEED = 0.817
    def scaleVel(self, left_vel, right_vel):
        # Scale from 0 to 1
        left_scale = left_vel / self.LEFT_MAX_SPEED 
        right_scale = right_vel / self.RIGHT_MAX_SPEED

        return left_scale, right_scale


#Create callback function here and drive motor based on message received
    def twistCallback(self, twist):
        left_vel, right_vel = self.fwdkin(twist)
        left_scale, right_scale = self.scaleVel(left_vel, right_vel)
        self.setWheelsSpeed(left_scale, right_scale)



#Initialize ROS node, subscriber and make the program run indefinitely
    def setupROS(self):
        rospy.Subscriber("/cmd_vel", Twist, self.twistCallback)

if __name__ == '__main__':
    rospy.init_node('duckiebot_motors', anonymous=False)
    obj = DaguWheelsDriver(debug=True)
    rospy.on_shutdown(obj.Shutdown)
    rospy.spin()



'''
#####################Example:
# Comment out after starting writing service
obj=DaguWheelsDriver()              #Create a DaguWheelsDriver object 
obj.setWheelsSpeed(0.5,0.5)         #Drive duckiebot straight
time.sleep(2)                       #for 2 seconds
obj.setWheelsSpeed(0.0,0.0)         #Stop duckiebot



###########stop the vehicle when program exits
obj.setWheelsSpeed(0.0,0.0)         #Stop duckiebot
'''
