import torch
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
 
# Load your model
try:
    model = torch.load('/home/robosub/subcode23/src/main/models/robosub_most_recent.pt')
except:
    rospy.signal_shutdown("failed to open model")

try:
    model.eval()
except:
    rospy.signal_shutdown("failed to eval model")

rospy.init_node('ml_process', anonymous=True)
motor_instruct = rospy.Publisher("motor_director", String, queue_size=10)


# Set up the camera
cap = cv2.VideoCapture(0)
 
# Labels
labels = {
    4: 'gate_rouge',
    8: 'buoy'
}
 
 
def preprocess_image(image):
    image = cv2.resize(image, (224, 224))  # Adjust based on your model's input size
    image = np.transpose(image, (2, 0, 1))  # Convert to channels first format
    image = torch.tensor(image).float()
    image = image.unsqueeze(0)  # Add batch dimension
    return image
 
def predict(image):
    with torch.no_grad():
        output = model(image)
    return output
 
def move_robot(displacement_x, displacement_y):
    threshold = 20  # Sensitivity threshold
 
    if abs(displacement_x) > threshold:
        if displacement_x > 0:
            rospy.log("Move Right")
            # Add your code to move the robot right
            motor_instruct.publish("Strafe_R")

        else:
            rospy.log("Move Left")
            # Add your code to move the robot left
            motor_instruct.publish("Strafe_L")
 
    if abs(displacement_y) > threshold:
        if displacement_y > 0:
            rospy.log("Move Down")
            # Add your code to move the robot backward
            motor_instruct.publish("Dive")
        else:
            rospy.log("Move Up/Forward")
            # Add your code to move the robot forward
            motor_instruct.publish("Climb")
def main():
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame")
            continue
        preprocessed_image = preprocess_image(frame)
        output = predict(preprocessed_image)
        predicted_class = torch.argmax(output, dim=1).item()
 
        if predicted_class in [4, 8]:  # gate_rouge or buoy
            rospy.log(f'Detected: {labels[predicted_class]}')
 
            # Assume the output includes bounding box coordinates, e.g., (x_min, y_min, x_max, y_max)
            # Example: if your model outputs bounding box along with class
            bounding_box = output[1][0]  # This is hypothetical and depends on your model's structure
            x_min, y_min, x_max, y_max = bounding_box
 
            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
 
            image_height, image_width = frame.shape[:2]
            image_center_x = image_width / 2
            image_center_y = image_height / 2
 
            displacement_x = center_x - image_center_x
            displacement_y = center_y - image_center_y
 
            move_robot(displacement_x, displacement_y)
        else:
            rospy.log('No target detected')
 
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

