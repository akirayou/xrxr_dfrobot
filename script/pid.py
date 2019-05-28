#!/usr/bin/env python

KILL_TIMER=0.5
PID_I_LIMIT=1.0
from drv8830 import Motor
def motor(l,r):
    motor.drv.drive(0,-l)
    motor.drv.drive(1,-r)
motor.drv=Motor()


class PID:
    def __init__(self):
        self.P=0
        self.I=0
        self.D=0
        self.target=0
        self.out=0
        self.E=0 #just for monitoring

        self._i=0
        self._oldE=0
        self._oldTime=None

    def stop(self,now = None):
        self._oldE=0
        self._i=0
        self.out=0
        if(not now==None):self._oldTime=now
        return 0

    def step(self,y,now):
        if(self._oldTime==None ): return self.stop(now)
        dt=(now-self._oldTime).to_sec()
        self._oldTime=now
        if(dt > KILL_TIMER): return self.stop(now)
        self.E=e=self.target-y
        d=(e-self._oldE)/dt
        self._i+=e*dt
        if(1e-10<self.I):
            i_limit=PID_I_LIMIT/self.I
            self._i=max(min(self._i,i_limit), -i_limit)
        self.out=self.P*e + self.I*self._i +self.D*d
        self._oldE=e
        return self.out

PID_v=PID()
PID_r=PID()


import rospy
import rosparam
from nav_msgs.msg import Odometry
import math
import numpy as np
import dynamic_reconfigure.client
from geometry_msgs.msg import Twist

last_odom_time=rospy.Time(0)
#global for moitoring
theV = None
theR = None
motorL=None
motorR=None
odomDelay=None
def have_odom():
    global odomDelay
    odomDelay=(rospy.get_rostime()-last_odom_time).to_sec()
    return odomDelay < KILL_TIMER
def xyz_to_np(i):
    return np.array([i.x,i.y,i.z])
def sub_odom(msg):
    global theV,theR,motorR,motorL,last_odom_time
    theV=msg.twist.twist.linear
    theR=msg.twist.twist.angular
    Yv=math.copysign(math.sqrt(theV.x**2+theV.z**2),theV.x)
    Yr=math.copysign(math.sqrt(theR.x**2+theR.z**2),theR.z)
    last_odom_time=msg.header.stamp
    v=PID_v.step(Yv,last_odom_time)
    v=PID_r.step(Yr,last_odom_time)
    if(abs(PID_v.target)<0.001 and abs(PID_r.target)<0.001 ):
        PID_v.stop(last_odom_time)
        PID_r.stop(last_odom_time)
        v=0
        r=0
        
    v=PID_v.out
    r=PID_r.out
    motorR=v+r
    motorL=v-r
    
    motor(motorL,motorR)


def sub_twist(msg):
    #rospy.loginfo("sub_twist: %s",str(msg))
    PID_v.target=msg.linear.x
    PID_r.target=msg.angular.z
    if(abs(PID_v.target)<0.001 and abs(PID_r.target)<0.001 ):
        PID_v.stop()
        PID_r.stop()
        motor(0,0)

import json
def cfg_callback(config):
    rospy.loginfo(config)
    PID_r.P=config.Rz_P
    PID_r.I=config.Rz_I
    PID_r.D=config.Rz_D
    PID_v.P=config.Vx_P
    PID_v.I=config.Vx_I
    PID_v.D=config.Vx_D


RATE = None
def wait_odom():
    rospy.loginfo("wait odom")
    while not rospy.is_shutdown():
        if(have_odom()):break
        RATE.sleep()
    rospy.loginfo("done")





if __name__ == "__main__":
    rospy.init_node('xrxr_pid')
    cfg = dynamic_reconfigure.client.Client("xrxr_dfrobot_cfg", timeout=6, config_callback=cfg_callback)
    RATE=rospy.Rate(2)
    odom = rospy.get_param('~odom', '/camera/odom/sample')
    rospy.loginfo("subscribe : %s",odom)
    twist = rospy.get_param('~cmd_vel', '/cmd_vel')
    rospy.loginfo("subscribe : %s",twist)
    rospy.Subscriber(odom,Odometry,sub_odom)
    rospy.Subscriber(twist,Twist,sub_twist)
    wait_odom()
    while not rospy.is_shutdown():
        RATE.sleep()
        rospy.loginfo( "Vx:%03f - %03f \t Rz:%03f - %03f \t Motor:%03f , %03f  ",
            PID_v.target, PID_v.E ,PID_r.target, PID_r.E,motorL,motorR)
        if(not have_odom()):
            #motor()function have big delay on Error state. (such as no power)
            #In case of that, subscriber loop (sub_odom) will not run correctory
            rospy.logerr("No odometry stop %f or motor driver error",odomDelay)
            PID_v.stop()
            PID_r.stop()
            motor(0,0)
