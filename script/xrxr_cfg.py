#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from xrxr_dfrobot.cfg import XRXRConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("xrxr_config", anonymous = False)

    srv = Server(XRXRConfig, callback)
    rospy.spin()