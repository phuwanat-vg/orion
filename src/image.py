#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class imageProc():

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image, self.camera_callback)
        print(" << Subscribe image from camera")
        
    def camera_callback(self,data):
        # Connect ROS with OpenCV via ROS cv_bridge package
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
      
        cv2.imshow("Image Window",cv_image)
        cv2.waitKey(1)

def main():
     
     rospy.init_node("image_proc_node", anonymous=True)
     ip = imageProc()
     rospy.spin()
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

