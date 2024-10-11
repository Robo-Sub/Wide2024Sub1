import cv2
import numpy as np
import rospy

rospy.init_node("classic_cam")
rate = rospy.Rate(3)

def define_color_ranges():
    # Red color range in HSV
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([179, 255, 255])
    
    return (red_lower1, red_upper1, red_lower2, red_upper2)

def process_image(image):
    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Get color ranges
    red_lower1, red_upper1, red_lower2, red_upper2 = define_color_ranges()
    
    # Create masks for RED color
    mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = mask1 | mask2
    
    # Find contours in the red mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        rospy.loginfo("Detected Red Color!")  # rospy.loginfo to the terminal if red is detected
    
    for contour in contours:
        # Get bounding box for each contour
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
    return image

def main():
    camera = cv2.VideoCapture(0)
    
    if not camera.isOpened():
        print("Failed to open camera")
        return

    while not rospy.is_shutdown():
        ret, frame = camera.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        processed_frame = process_image(frame)
        
        cv2.imshow('Captured Frame', frame)
        cv2.imshow('Processed', processed_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        rate.sleep()

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass