#! /usr/bin/env python3 

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Detect:
    def __init__(self):
        rospy.init_node("detect", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.res = None
        self.barells_found = 0
    
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_range = np.array([5, 100, 100])
        upper_range = np.array([15, 255, 255])
        
        mask = cv2.inRange(hsv_image, lowerb=lower_range, upperb=upper_range)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:    
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w ,h = cv2.boundingRect(max_contour)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
        self.res = cv_image

    def run(self):
        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.res is not None:
                cv2.imshow("Detection", self.res)
                cv2.waitKey(1)
            rate.sleep()
        cv2.destroyAllWindows()
     
            
if __name__ == "__main__":
    detect = Detect()
    detect.run()
    
    
