#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class redTraffic():

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image, self.camera_callback)
        print(" << Subscribe image from camera")
        self.detect_pub = rospy.Publisher("red_light", String, queue_size = 3)
        
    def camera_callback(self,data):
        # Connect ROS with OpenCV via ROS cv_bridge package
        try:
            cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0,100,100])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([160,100,100])
        upper_red2 = np.array([180,255,255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maskr = cv2.add(mask1, mask2)

        r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80,
            param1=30, param2=15, minRadius=0, maxRadius=80)

        font = cv2.FONT_HERSHEY_SIMPLEX
        
        detect_status = String()
        # traffic light detect

        if r_circles is not None:
            detect_status.data = "detect"
            r_circles = np.uint16(np.around(r_circles))
            for i in r_circles[0,:]:
                # draw the outer circle
                cv2.circle(cv_image, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                cv2.circle(maskr, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                cv2.putText(cv_image,'RED',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)
        else:
            detect_status.data = "no"
        self.detect_pub.publish(detect_status)
        cv2.imshow("Image Window",cv_image)
        cv2.imshow("HSV",hsv)
        cv2.imshow("MASK",maskr)
        cv2.waitKey(1)

def main():
     
     rospy.init_node("red_traffic_light_node", anonymous=True)
     rt = redTraffic()
     rospy.spin()
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

