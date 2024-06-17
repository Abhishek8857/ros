#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
import cv_bridge

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cv_image = None
        
        
    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


    def run(self):
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                cv2.imshow("Camera Feed", self.cv_image)
                cv2.waitKey(1)
            rate.sleep()
        cv2.destroyAllWindows()
        
        
def main():
    rospy.init_node("follow", anonymous=True)
    display = Follower()
    display.run()
        

if __name__ == '__main__':
    main()    
        
                