#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import dynamic_reconfigure.client


class Picker(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.camera_callback)
        print(" << Subscribe image from camera")
        self.hue_l=29
        self.hue_h=330
        self.saturation_l=86
        self.saturation_h=255
        self.lightness_l=86
        self.lightness_h=255
       

    def camera_callback(self,data):
        
        try:
            cv_image=self.bridge.compressed_imgmsg_to_cv2(data,"bgr8")
          
        except CvBridgeError as e:
            print(e)

        blob_params = None
        #blur to reduce noise
        #blurred = cv2.GaussianBlur(cv_image, (11,11),0)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV).astype(np.float)      

        lower_color = np.array([self.hue_l,self.saturation_l,self.lightness_l])
        upper_color = np.array([self.hue_h,self.saturation_h,self.lightness_h])

        mask = cv2.inRange(hsv,lower_color,upper_color)  

         #find contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
       

        #erode
        mask = cv2.erode(mask, None, iterations=2)

        #dilate area larger
        mask = cv2.dilate(mask, None, iterations=2)
        
      
        cv2.imshow("Image Window",cv_image)
        cv2.imshow("MASK",mask)
       
   
     
        cv2.waitKey(1)
    
    def cfg_callback(self,config):

        self.hue_l = config.hue_l
        self.hue_h = config.hue_h
        self.saturation_l = config.saturation_l
        self.saturation_h = config.saturation_h
        self.lightness_l = config.lightness_l
        self.lightness_h = config.lightness_h
        print("hue_l :{} hue_l :{} sat_l :{} sat_h :{} v_l :{} v_h :{}".format(self.hue_l,self.hue_h,self.saturation_l,self.saturation_h\
        ,self.lightness_l,self.lightness_h))

def main():
     
     rospy.init_node("color_picker_client_node", anonymous=True)
   
     color_picker=Picker()
     client = dynamic_reconfigure.client.Client("color_picker_server_node/dyn_rec", timeout=100, config_callback=color_picker.cfg_callback)
     
     try:
       
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

