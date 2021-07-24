#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage


class BallTrack(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.camera_callback)
        print(" << Subscribe image from camera")
        self.vel_pub = rospy.Publisher("orion/cmd_vel", Twist, queue_size = 5)
        print(" >> Publish the velocity ")
        self.vel = Twist()
        

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        

    def drive(self,vx,wz) :
        self.vel.linear.x = vx
        self.vel.angular.z = wz

    def camera_callback(self,data):
        # Connect ROS with OpenCV via ROS cv_bridge package
        try:
            cv_image=self.bridge.compressed_imgmsg_to_cv2(data,"bgr8")
            self.vel_pub.publish(self.vel)
        except CvBridgeError as e:
            print(e)

        blob_params = None
        #blur to reduce noise
        #blurred = cv2.GaussianBlur(cv_image, (11,11),0)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV).astype(np.float)      

        lower_color = np.array([0,94,53])
        upper_color = np.array([360,255,255])

        mask = cv2.inRange(hsv,lower_color,upper_color)  

         #find contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
       

        #erode
        mask = cv2.erode(mask, None, iterations=2)

        #dilate area larger
        mask = cv2.dilate(mask, None, iterations=2)
       
        #circle
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 3, 500, minRadius = 10, maxRadius = 200, param1 = 100, param2 = 60)
        object_h_pixel = 0
       
        if circles is not None:
            circles = np.round(circles[0,:]).astype("int")
            for (x,y, radius) in circles:
                cv2.circle(cv_image, (x,y), radius, (0,255,0),4)
                if radius>20 :
                    object_h_pixel = x
                else:
                    object_h_pixel = 0

                print("x-pixel:{}  y:pixel{} radius: {}".format(x,y,radius))
        #initialize
        center = None

        #image size
        rows = cv_image.shape[0]
        cols = cv_image.shape[1]
        size = min([rows,cols])
        center_x = int(cols/2.0)
        center_y = int(rows/2.0)
      
        threshold = 80
        left_bound = int(center_x-threshold)
        right_bound = int(center_x+threshold)        
        vx =0
        wz =0
        #Cannot detect ball
        if object_h_pixel == 0 or radius > 60:
            hdif = 0
            self.stop()
        elif (object_h_pixel < left_bound and radius > 30 and radius < 60):
            
            #vx = 0
            wz = -4.3
            self.drive(vx,wz)

        elif (object_h_pixel > right_bound and radius > 30 and radius < 60):
      
            #vx = 0
            wz = 4.3
            self.drive(vx,wz)
        else:
            vx = 0.2
            #wz = 0.0
            self.drive(vx,wz)
      
        cv2.imshow("Image Window",cv_image)
        cv2.imshow("MASK",mask)
        #cv2.imshow("HSV",hsv)
      
   
     
        cv2.waitKey(1)

def main():
     
     rospy.init_node("line_track_node", anonymous=True)
     ball_tracking_object=BallTrack()
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

