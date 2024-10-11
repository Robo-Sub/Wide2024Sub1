#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

def get_arrow_direction(contour):
    # Approximate the contour
    epsilon = 0.03 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    if len(approx) == 7:
        # Find the bounding box of the contour
        x, y, w, h = cv2.boundingRect(contour)

        # Split the contour into left and right halves
        left_half = [pt for pt in approx if pt[0][0] < x + w / 2]
        right_half = [pt for pt in approx if pt[0][0] >= x + w / 2]

        # Check if the left half has more points than the right half
        if len(left_half) > len(right_half):
            return "left"
        else:
            return "right"
    return None

class ArrowDetectionNode:
    def __init__(self):
        rospy.init_node('arrow_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.direction_pub = rospy.Publisher("/arrow_direction", String, queue_size=10)
        rospy.loginfo("Arrow Detection Node Initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                direction = get_arrow_direction(contour)
                if direction:
                    # Draw the contour and direction on the image
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.putText(cv_image, direction, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                    self.direction_pub.publish(direction)

            # Display the processed image
            cv2.imshow('Arrow Direction', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s" % str(e))

if __name__ == "__main__":
    try:
        ArrowDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
