#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage


def callback(config,level):
    rospy.loginfo("""Reconfigure request: {hue_l}, {hue_h},{saturation_l},{saturation_h},{lightness_l},{lightness_h}""".format(**config))
    print("{}".format(config))   
    return config
          

if __name__ =='__main__':
     rospy.init_node("color_picker_server_node", anonymous=False)
     print("Rqt reconfigure server start")
     
     ddrec = DDynamicReconfigure("dyn_rec")
     ddrec.add_variable("hue_l", "Hue_lower", 0,0,360)
     ddrec.add_variable("hue_h", "Hue_upper", 360,0,360)
     ddrec.add_variable("saturation_l", "Saturation_lower", 0,0,255)
     ddrec.add_variable("saturation_h", "Saturation_upper", 255,0,255)
     ddrec.add_variable("lightness_l", "Lightness_lower", 0,0,255)
     ddrec.add_variable("lightness_h", "Lightness_upper", 255,0,255)

     
     ddrec.start(callback)
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
        

