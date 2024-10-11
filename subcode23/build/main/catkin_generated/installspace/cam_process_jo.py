import cv2
import rospy
import numpy as np
from std_msgs.msg import Bool, String


# Webcamera no 0 is used to capture the frames
cap = cv2.VideoCapture(0)

# Color publisher
rospy.init_node('cam_process')
color_pub = rospy.Publisher("camera_color", String, queue_size=10)
edge_pub = rospy.Publisher("edge_detect", Bool, queue_size=10)

# Get parameter values
frequency = rospy.get_param('~/frequency', 3)

#global vars
rate = rospy.Rate(3)
color = 0

# Define color BGR ranges
color_ranges = {
    "red": ([9, 13, 112], [41, 35, 171], 1),
    "green": ([95, 100, 29], [160, 181, 109], 4),
    "blue": ([90, 50, 20], [150, 120, 80], 5),
    "orange": ([7, 40, 136], [49, 104, 193], 2),
    "yellow": ([10, 100, 134], [46, 152, 186], 3),
    "purple": ([80, 30, 40], [152, 84, 128], 6),
    "black": ([24, 19, 17], [70, 70, 60], 7)
}

# Check if the video capture object was initialized correctly.
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture frame")
        continue

    # Create a downscaled frame
    size = frame.shape
    frame1 = frame
#    frame1 = np.zeros((int(size[0] / 10), int(size[1] / 10), size[2]), dtype=np.uint8)
#    for i in range(0, int(size[0] / 10)):
#        for j in range(0, int(size[1] / 10)):
#            frame1[i, j] = frame[10 * i, 10 * j]

    # Calculate dem
    dem = 255 * frame1.shape[0] * frame1.shape[1]

    # Threshold for color detection
    thresh = 0.2

    max_reading = 0
    detected_color = None
    detected_color_value = None

    # Find color with the highest reading
    for color_name, (lower, upper, color_value) in color_ranges.items():
        mask = cv2.inRange(frame1, np.array(lower), np.array(upper))
        color_t = np.sum(mask) / dem
        rospy.loginfo(f'{color_name.capitalize()}: {color_t}')

        if color_t > max_reading:
            max_reading = color_t
            detected_color = color_name
            detected_color_value = color_value

    if max_reading > thresh:
        if detected_color == "orange":
            # Increase motor speed by 10 when orange is detected
            rospy.loginfo(f"Detected orange with reading {max_reading}")
        elif detected_color == "purple":
            # Decrease motor speed by 10 when purple is detected
            rospy.loginfo(f"Detected purple with reading {max_reading}")
        elif detected_color == "yellow":
            # Perform operations only if yellow has the highest reading
            rospy.loginfo(f"Detected yellow with reading {max_reading}")
        elif detected_color == "green":
            # Perform operations only if green has the highest reading
            rospy.loginfo(f"Detected green with reading {max_reading}")
        else:
            rospy.loginfo("No relevant color detected above the threshold")

        if detected_color_value is not None:
            color_pub.publish(detected_color)
    else:
        color_pub.publish("None")

    # Rotate frame and handle any interruptions
    frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
    cv2.imshow('frame1', frame1)

    # Break loop if the ESC key is pressed
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    rate.sleep()


    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # If frame is read correctly, ret is True
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection method on the image
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # This returns an array of r and theta values
    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)

    # The below for loop runs till r and theta values are in the range of the 2d array
    if lines is not None:
        for r_theta in lines:
            arr = np.array(r_theta[0], dtype=np.float64)
            r, theta = arr
            # Stores the value of cos(theta) in a
            a = np.cos(theta)

            # Stores the value of sin(theta) in b
            b = np.sin(theta)

            # x0 stores the value rcos(theta)
            x0 = a * r

            # y0 stores the value rsin(theta)
            y0 = b * r

            # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
            x1 = int(x0 + 1000 * (-b))

            # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
            y1 = int(y0 + 1000 * (a))

            # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
            x2 = int(x0 - 1000 * (-b))

            # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
            y2 = int(y0 - 1000 * (a))

            # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
            # (0,0,255) denotes the colour of the line to be drawn. In this case, it is red.
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    rate.sleep()

    
    # Display the resulting frame
    #cv2.imshow('Frame', frame)

    # Press 'q' on the keyboard to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the video capture object
cap.release()
# Close all OpenCV windows
cv2.destroyAllWindows()
