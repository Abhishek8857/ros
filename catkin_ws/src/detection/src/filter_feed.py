#! /usr/bin/env python3

import rospy
import cv2
import numpy
import cv_bridge
from sensor_msgs.msg import Image

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cv_image = None
        self.yellow_mask = None
        self.white_mask = None
        self.combined_mask = None

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        
        lower_yellow = numpy.array([0, 0, 0])
        upper_yellow = numpy.array([180, 255, 50])
        self.yellow_mask =  cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 55, 255])
        self.white_mask = cv2.inRange(hsv, lower_white, upper_white)
        self.combined_mask = cv2.bitwise_or(self.yellow_mask, self.white_mask)
    
    
        cv2.imshow("Yellow Mask", self.yellow_mask)
        cv2.imshow("White Mask", self.white_mask)
        cv2.imshow("Combined Mask", self.combined_mask)
        cv2.waitKey(1)
        
        
    def run(self):
        cv2.namedWindow("Filtered Feed", cv2.WINDOW_NORMAL)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                height, width, _ = self.cv_image.shape
                
                top = int(0.75 * height)
                bot = top + 20
                
                self.combined_mask[0:top, 0:width] = 0
                self.combined_mask[bot:height, 0:width] = 0
                
                cv2.imshow("Cropped Mask", self.combined_mask)

                mom = cv2.moments(self.combined_mask)
                if mom["m00"] > 0:
                    c_x = int(mom["m10"]/mom["m00"])
                    c_y = int(mom["m01"]/mom["m00"])
                    cv2.circle(self.cv_image, (c_x, c_y), 20, (0, 0, 255), -1)
                    
                    rospy.loginfo(f"Centroid coordinates: ({c_x}, {c_y})")
                else:
                    # Debugging: Indicate no valid centroid was found
                    rospy.loginfo("No valid centroid found")
                    
                    
                cv2.imshow("Filtered Feed", self.combined_mask)
                cv2.waitKey(1)
            rate.sleep()
        cv2.destroyAllWindows()


def main():
    rospy.init_node("Follower", anonymous=True)
    display = Follower()
    display.run()
    
    
if __name__ == "__main__":
    main()
