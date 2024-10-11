import cv2
import rospy
import numpy as np
from std_msgs.msg import Bool, String
import os

def create_color_map(frame):
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Define the number of bins for the histogram
    bins = [180, 256]  # Hue and Saturation
    hist_range = [0, 180, 0, 256]
    
    # Compute the histogram
    hist = cv2.calcHist([hsv_frame], [0, 1], None, bins, hist_range)
    hist = cv2.normalize(hist, hist).flatten()
    
    dim = hsv_frame.shape

    # Create a color map to display
    color_map = np.zeros((dim[0], dim[1], 3), dtype=np.uint8)

    # Populate the color map
    for i in range(dim[0]):
        for j in range(dim[1]):
            color = np.array([hsv_frame[i, j, 0], hsv_frame[i, j, 1], 255], dtype=np.uint8)
            color_map[i, j, :] = color
    
    return color_map, hist

def calculate_center(lines):
    if lines is None:
        return None
    x_sum = 0
    y_sum = 0
    count = 0
    for r_theta in lines:
        arr = np.array(r_theta[0], dtype=np.float64)
        r, theta = arr
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * r
        y0 = b * r
        x_sum += x0
        y_sum += y0
        count += 1
    if count > 0:
        x_center = x_sum / count
        y_center = y_sum / count
        return (x_center, y_center)
    else:
        return None

def find_right_angles(line_list):
    angles = []
    if line_list is not None:
        for i in range(len(line_list)):
            for j in range(i + 1, len(line_list)):
                rho1, theta1 = line_list[i][0]
                rho2, theta2 = line_list[j][0]
                angle = np.abs(theta1 - theta2)
                angle = np.degrees(angle)
                if np.isclose(angle, 90, atol=15):
                    angles.append((rho1, theta1, rho2, theta2, line_list[i], line_list[j]))
    return angles

cap = cv2.VideoCapture(0)
frame_path = '/home/robosub/subcode23/src/main/collected_pics/'

#global vars
color = 0
frames_saved = 0

# Define color BGR ranges
color_ranges = {
    "orange": ([7, 40, 136], [49, 104, 193], 2),
}

def save_frame(i):
    frame_name = f'frame_{i}.jpg'
    os.chdir(frame_path)
    cv2.imwrite(frame_name, frame)

# Webcamera no 0 is used to capture the frames

# Color publisher
rospy.init_node('cam_process')
color_pub = rospy.Publisher("camera_color", String, queue_size=10)
edge_pub = rospy.Publisher("edge_detect", Bool, queue_size=10)

rate = rospy.Rate(3)

# Check if the video capture object was initialized correctly.
if not cap.isOpened():
    rospy.log("Error: Could not open video stream.")
    exit()

while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture frame")
        continue

    color_map, hist = create_color_map(frame)
    #cv2.imshow('Color Map', color_map)

    # Create a downscaled frame
#    size = frame.shape
#    frame1 = frame
##    frame1 = np.zeros((int(size[0] / 10), int(size[1] / 10), size[2]), dtype=np.uint8)
##    for i in range(0, int(size[0] / 10)):
##        for j in range(0, int(size[1] / 10)):
##            frame1[i, j] = frame[10 * i, 10 * j]
#
#    # Calculate dem
#    dem = 255 * frame1.shape[0] * frame1.shape[1]
#
#    # Threshold for color detection
#    thresh = 0.2
#
#    max_reading = 0
#    detected_color = None
#    detected_color_value = None
#
#    # Find color with the highest reading
#    for color_name, (lower, upper, color_value) in color_ranges.items():
#        mask = cv2.inRange(frame1, np.array(lower), np.array(upper))
#        color_t = np.sum(mask) / dem
#        rospy.loginfo(f'{color_name.capitalize()}: {color_t}')
#
#        if color_t > max_reading:
#            max_reading = color_t
#            detected_color = color_name
#            detected_color_value = color_value
#
#    if max_reading > thresh:
#        if detected_color == "orange":
#            # Increase motor speed by 10 when orange is detected
#            rospy.loginfo(f"Detected orange with reading {max_reading}")
##        elif detected_color == "purple":
##            # Decrease motor speed by 10 when purple is detected
##            rospy.loginfo(f"Detected purple with reading {max_reading}")
##        elif detected_color == "yellow":
##            # Perform operations only if yellow has the highest reading
##            rospy.loginfo(f"Detected yellow with reading {max_reading}")
##        elif detected_color == "green":
##            # Perform operations only if green has the highest reading
##            rospy.loginfo(f"Detected green with reading {max_reading}")
#        else:
#            rospy.loginfo("No relevant color detected above the threshold")
#
#        if detected_color_value is not None:
#            color_pub.publish(detected_color)
#    else:
#        color_pub.publish("None")

    #Rotate frame and handle any interruptions
    #frame1 = cv2.rotate(frame1, cv2.ROTATE_180)
    #cv2.imshow('frame1', frame1)

    save_frame(frames_saved)
    frames_saved = frames_saved+1


    # Break loop if the ESC key is pressed
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    rate.sleep()

    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    if not ret:
        rospy.log("Failed to grab frame")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Detect lines in the image
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

    frame = cv2.cvtColor(edges,cv2.COLOR_GRAY2RGB)

    angles = find_right_angles(lines)
    # Draw the detected lines on the original frame
    if angles is not None:
        for linesf in angles:
            arr = np.array(linesf[4][0], dtype=np.float64)
            r, theta = arr
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * r
            y0 = b * r
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            arr = np.array(linesf[5][0], dtype=np.float64)
            r, theta = arr
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * r
            y0 = b * r
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Calculate the center of the detected lines
    center = calculate_center(lines)

    # Find right angles from the detected lines
    right_angles = find_right_angles(lines)
    if right_angles:
        rospy.log("Right angles found:", right_angles)
        for angle in right_angles:
            rho1, theta1, rho2, theta2, temp, temp = angle
            rospy.log(f"Line 1: rho={rho1}, theta={theta1}")
            rospy.log(f"Line 2: rho={rho2}, theta={theta2}")

    

    # Display the frame with lines
    #cv2.imshow('edges', frame)

    rate.sleep()
    

    # Press 'q' on the keyboard to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the video capture object
cap.release()
# Close all OpenCV windows
cv2.destroyAllWindows()
