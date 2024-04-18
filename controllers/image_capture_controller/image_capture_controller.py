# Import the necessary Webots, PIL, and NumPy modules
from controller import Robot, Camera
from PIL import Image
import numpy as np
import cv2

# Create an instance of the Robot class
robot = Robot()

# Get the camera device
camera = robot.getDevice("camera")  

# Enable the camera
camera.enable(64)  # The argument is the refresh rate (in milliseconds)

# Main control loop
while robot.step(64) != -1:  # Use an appropriate time step
    # Get the camera image
    image = camera.getImage()
    
    # Convert the camera image to a NumPy array as we are using cv2 in all the steps ahead
    image_np = np.frombuffer(image, np.uint8)
    
    # Reshape the NumPy array to the dimensions of the camera imag
    image_np = image_np.reshape((camera.getHeight(), camera.getWidth(), 4))
    
    # Converting the arena image to grayscale
    large_gray = cv2.cvtColor(image_np, cv2.COLOR_BGRA2GRAY)
    
    #template is the robot which will be searched in the arena image,
    #it is imported from the drive as we already have top image of the robot , the image is stored in the image_capture_controller folder in controllers of the project.
    template = cv2.imread(r"C:\Users\91809\Downloads\project\project\controllers\image_capture_controller\firebird_6.png")
    # template is converted into grayscale 
    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    #getting the dimentions of the robot template
    w, h = template_gray.shape[::-1]
    # searching the grey scaled arena for the grey scaled robot
    res = cv2.matchTemplate(large_gray, template_gray, cv2.TM_CCOEFF_NORMED)
    #gettng the location of the matched robot on the arena
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    #replacing the robot in the arena witha white rectangle
    cv2.rectangle(large_gray, top_left, bottom_right, (255, 255, 255), thickness=cv2.FILLED)
    
    #saving the image of arena in greyscale with removed robot, as robot has been replaced with white rectangle
    cv2.imwrite(r"C:\Users\91809\Downloads\project\project\controllers\image_capture_controller\robot_removed_arena.png", large_gray)
    
    
    #breaking the loop, as for now we are required to write the image once at the start of the operation only. it can be changed to require frequency
    break

#disabling the camera as it is not required now
camera.disable()
