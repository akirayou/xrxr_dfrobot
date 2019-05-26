#!/usr/bin/env python
from drv8830 import Motor
def motor(l,r):
    motor.drv.drive(0,-l)
    motor.drv.drive(1,-r)
motor.drv=Motor()

import rospy
import rosparam
from nav_msgs.msg import Odometry
import math
import numpy as np
haveOdom = False
theV = None
theR = None
thePV = None
thePR = None
def xyz_to_np(i):
    return np.array([i.x,i.y,i.z])
def sub_odom(msg):
    global haveOdom,theV,theR,thePV,thePR
    theV=xyz_to_np(msg.twist.twist.linear)
    theR=xyz_to_np(msg.twist.twist.angular)
    thePV=xyz_to_np(msg.pose.pose.position)
    thePR=xyz_to_np(msg.pose.pose.orientation)
    haveOdom=True

RATE = None
def wait_odom():
    while not rospy.is_shutdown():
        if(haveOdom):break
        RATE.sleep()

def check_rot():
    motor(-0.3*0,0.3)
    for i in range(3):
        RATE.sleep()
    startPR=thePR
    r=np.zeros(3)
    count=0
    print("startPR",startPR)
    startTime = rospy.get_rostime()
    for i in range(10):
        RATE.sleep()
        r+=theR
        count+=1
        print(theR)
    for i in range(100):
        RATE.sleep()
        print(theR)
        r+=theR
        count+=1
        diff=thePR-startPR
        if(abs(diff[2])<0.1):break
    print("endPR",thePR)
    motor(0,0)
    endTime = rospy.get_rostime()
    print (endTime-startTime).to_sec()
    print 2*math.pi/(endTime-startTime).to_sec()
    r/=count
    print(r)
    print(np.linalg.norm(r))

def check_forward():
    motor(0.3,0.3)
    for i in range(10):
        RATE.sleep()
        print(theV)


def mesure(l,r):
    motor(l,r)
    for i in range(10):
        RATE.sleep()
    v=np.zeros(3)
    r=np.zeros(3)
    for i in range(20):
        RATE.sleep()
        v+=theV
        r+=theR
    v/=30
    r/=30
    motor(0,0)
    return (v,r)

def to_vx_rz(m):
    #camera can tilt up or down
    #camera can offset forward position, it means you must ignore y direction velocity
    vx=math.sqrt(m[0][0]**2+m[0][2]**2)
    rz= np.linalg.norm(m[1][0])     #(m[1][2]**2+m[1][1]**2)**2
    return (vx,rz)

def to_L(vx,rx):
    return vx*2/rx

if __name__ == "__main__":
    rospy.init_node('xrxr_carib')
    RATE=rospy.Rate(10)
    odom = rospy.get_param('~odom', '/camera/odom/sample')
    rospy.loginfo("subscribe : %s",odom)
    rospy.Subscriber(odom,Odometry,sub_odom)
    rospy.loginfo("wait odom")
    wait_odom()
    #check_forward()
    check_rot()


    r =mesure(0.4,0)
    res=to_vx_rz(r)
    #print(r)
    rospy.loginfo("vx: %f, rz:%f ",res[0],res[1])
    l=to_L(*res)
    rospy.loginfo("extimated L  : %f",l)

    for i in range(30):
        RATE.sleep()

    r =mesure(0,0.4)
    res=to_vx_rz(r)
    #print(r)
    rospy.loginfo("vx: %f, rz:%f ",res[0],res[1])
    l=to_L(*res)
    rospy.loginfo("extimated L  : %f",l)


