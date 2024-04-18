
import heapq
import cv2
import numpy as np
import math
from controller import Robot

# Initialize the robot and its devices
robot = Robot()
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

#delaying the start of the robot by 1 second so that image is captured prior to calling the image 
robot.step(1000)
#Reading the robot removed arena
#robot removed arena is saved in the image_capture_cntrollers, so give the path by browsing it if it is not functioning properly
image = cv2.imread(r"C:\Users\91809\Downloads\project\project\controllers\image_capture_controller\robot_removed_arena.png")

#Crop and resize the image
x, y, w, h = 20, 20, 200, 200 
cropped_image = image[y:y+h, x:x+w]
resized_image = cv2.resize(cropped_image, (101, 101))
cv2.imwrite('resized_cropped_image.jpg',resized_image)

# Convert the resized image to a binary image
gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
_, binary_image = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
binary_image[binary_image == 0] = 1
binary_image[binary_image == 255] = 0

#Convert the binary image to list of strings
b = [str(row) for row in binary_image.tolist()]


#Define mapsize and gridsize for A* path planning
mapSize = 10
gridSize = 0.1


    
# Set the motors to rotate for ever
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
# But with no velocity
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Enable GPS and Compass sensors
sampling_period = 1 # in ms
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)
    
robot.step(1000) # take some dummy steps in environment for safe initialization of the initial heading and position

#Convert GPS Coordinates to grid coordinates
def gpsToMap(x, y):
    location_x = int((-x + mapSize / 2) / gridSize)
    location_y = int((y + mapSize / 2) / gridSize)
    return location_x, location_y

# Convert the map coordinates to GPS coordinates
def mapToGps(i, j):
    x = -1*((i ) * gridSize - mapSize / 2)
    y = ((j ) * gridSize - mapSize / 2)
    return x, y

# Define a Node class for A* algorithm
class Node:
    def __init__(self, x, y, cost=0, heuristic=0):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = None  # Add parent attribute

    #Comparision function for node while inserting in heapq
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

#Function to calculate manhattan distance between two nodes
def distance(node1, node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)

#Function for A-star path planning
def a_star(start, goal, grid):
    # Initialize the open set with the starting node
    open_set = [start]
    # Initialize the closed set as an empty set
    closed_set = set()

    # Continue until the open set is not empty
    while open_set:
        # Get the node with the lowest cost from the open set
        current = heapq.heappop(open_set)

        # Check if the current node is the goal
        if current.x == goal.x and current.y == goal.y:
            # Reconstruct and return the path from the goal to the start
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1] # Reverse the path to start from the beginning

        # Add the current node to the closed set
        closed_set.add((current.x, current.y))

        # Iterate over possible neighbor directions: up, down, left, right, and diagonals
        for i, j in [(1, 0), (-1, 0), (0, 1), (0, -1), (-1, -1), (-1, 1), (1, -1), (1,1)]:
            # Create a potential neighbor node
            neighbor = Node(current.x + i, current.y + j, current.cost + 1, distance(current, goal))

            # Check if the neighbor is within the grid boundaries and is not in the closed set
            if (
                0 <= neighbor.x < len(grid) and
                0 <= neighbor.y < len(grid[0]) and
                grid[neighbor.x][neighbor.y] == 0 and
                (neighbor.x, neighbor.y) not in closed_set
            ):
                # Add the neighbor to the open set with updated cost and heuristic values
                heapq.heappush(open_set, neighbor)
                # Set the parent of the neighbor to the current node
                neighbor.parent = current
    
    # If the open set becomes empty and the goal is not reached, return None (no path found)
    return None  # No path found



#Function to pad cells around obstacle coordinates in grid.
def populate_adjacent_cells(matrix):
    # Define directions for up, down, left, and right
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (-1, -1), (-1, 1), (1, -1), (1,1)]

    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            if matrix[i, j] == 1:
                # Check adjacent cells in all directions
                for dir_i, dir_j in directions:
                    new_i, new_j = i + dir_i, j + dir_j
                    if 0 <= new_i < matrix.shape[0] and 0 <= new_j < matrix.shape[1] and matrix[new_i, new_j] == 0:
                        # Populate adjacent cell with 3
                        matrix[new_i, new_j] = 3
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            matrix[i,j] = matrix[i,j]%2
# Call the function to populate adjacent cells


########## function to navigate the path generated by A* algorithm ################
def navigate(Path):
    m=1
    for coor in Path:
        
        targ_x,targ_y = mapToGps(coor[0], coor[1])
        # Get robot's heading in degree based on compass values
        def get_robot_heading(compass_value):
            rad = math.atan2(compass_value[0], compass_value[1])
            bearing = (rad - 1.5708) / math.pi * 180.0
            if bearing < 0.0:
                bearing = bearing + 360.0
            
            heading = 360 - bearing
            if heading > 360.0:
                heading -= 360.0
            return heading
        
        
        # Create an instance of robot
        # robot = Robot()
        
    
        left_motor = robot.getDevice('left wheel motor')
        right_motor = robot.getDevice('right wheel motor')
        

        left_motor.setPosition(float('inf'))
        right_motor.setPosition(float('inf'))

        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
        # sampling_period = 1 # in ms
        gps = robot.getDevice("gps")
        compass = robot.getDevice("compass")

        gps.enable(sampling_period)
        compass.enable(sampling_period)
        
        # robot.step(1000) # take some dummy steps in environment for safe initialization of the initial heading and position
        initial_gps_value = gps.getValues()
        initial_compass_value = compass.getValues()
        
        # General parameters
        max_speed = 2.0  # Angular speed in rad/s
        destination_coordinate = [targ_x, targ_y] # Target position 
        distance_threshold = 0.1  # in meters
        angle_threshold = 1 # angle degree threshold to check if heading is in range
        
        # degree to target using direction vector
        direction_vector = [destination_coordinate[0] - initial_gps_value[0], 
                            destination_coordinate[1] - initial_gps_value[1]]
        degree_to_target = math.atan2(direction_vector[0], direction_vector[1]) * 180 / math.pi
        degree_to_target = round(degree_to_target) % 360
        
        # initial degree of robot
        initial_degree = round(get_robot_heading(initial_compass_value))
        
        # how much initial degree differs from target angle
        degree_diff = initial_degree - degree_to_target
        print(f'Target Angle: {degree_to_target}, Robot Heading: {initial_degree}')
        
        # condition for taking the shorter rotation
        if degree_diff < 0:
            rotate_right = True if abs(degree_diff) <= 180 else False
        else:
            rotate_right = False if abs(degree_diff) <= 180 else True
            
        # Rotate the robot to face the target angle
        while True:
            current_degree = get_robot_heading(compass.getValues()) # current robot angle
            diff_current_vs_initial_degree = abs(current_degree - degree_to_target) # difference of angle bettwen intial degree vs current degree of robot heading
            if rotate_right:
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(-max_speed)
            else:
                left_motor.setVelocity(-max_speed)
                right_motor.setVelocity(max_speed)
            robot.step()
            
            # If condition is met break the loop
            if diff_current_vs_initial_degree < angle_threshold:
                print("CurrentDegree",current_degree)
                print("Target_degree",degree_to_target)
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                robot.step(1000)  
                # print('Correct Heading.')
                break
            
        
        # Move the robot to the destination
        i=1
        
        reached_first_8 = False
        start_coordinate=[0,0]
        while True:
            current_coordinate = gps.getValues()
            gps_value = gps.getValues()
            current_gps = [0,0]
            for j in range(len(gps_value)-1):
                current_gps[j]=round(gps_value[j],2)
                x,y = gpsToMap(current_gps[0], current_gps[1])
            print("Current cell",x,y)
            print("Current gps", current_gps)   # current robot position via gps sensor
            for i in range(10):
                if(i==8 and reached_first_8==False):
                    for j in range(len(current_coordinate)-1):
                        start_coordinate[j]=round(current_coordinate[j],2)
                    reached_first_8=True
            distance_to_target_x = abs(current_coordinate[0] - destination_coordinate[0]) # difference in x of current vs destination position
            distance_to_target_y = abs(current_coordinate[1] - destination_coordinate[1]) # difference in y of current vs destination position
            distance_to_target = math.sqrt(abs(current_coordinate[0] - destination_coordinate[0])**2+abs(current_coordinate[1] - destination_coordinate[1])**2)#Calculating Euclidean distance
            # reach_condition = distance_to_target_x and distance_to_target_y < distance_threshold # did robot reach the target location?
            reach_condition = distance_to_target< distance_threshold
            # Check if the robot is already at the destination
            if reach_condition:
                gps_value = gps.getValues()
                current_gps = [0,0]
                for j in range(len(gps_value)-1):
                    current_gps[j]=round(gps_value[j],2)
                x,y = gpsToMap(current_gps[0], current_gps[1])
                # print('Destination Reached.',m," : ",x,y)
                m=m+1
            else:
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
            robot.step()
            
            # if reached break the loop
            if reach_condition:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                robot.step(1000)  
                print(current_coordinate)
                print('Finished.')
                break
    return 0
grid = binary_image

################# Main Loop #####################

#Define goal coordinates to be traversed in sequence.
destination = [ [1 , 0],[3, -2] , [-3.5, -3.5], [1 , 0]]


for goal in destination:

    initial_gps_value = gps.getValues()
    print(initial_gps_value)
    initial_compass_value = compass.getValues()
    start_coordinate=[0,0]
    for j in range(len(initial_gps_value)-1):
        start_coordinate[j]=round(initial_gps_value[j],2)

    print("New grid below----------------------")
    print(grid)
    start_x,start_y = gpsToMap(start_coordinate[0],start_coordinate[1])#Getting the grid coordinate from gps coordinate
    print(start_x,start_y)
    print(grid[start_x][start_y])
    end_x,end_y = gpsToMap(goal[0],goal[1])
    print(end_x,end_y)
    print(grid[end_x][end_y])
    start_node = Node(start_x,start_y )
    goal_node = Node(end_x,end_y)
    print("Called A*")
    populate_adjacent_cells(grid)
    # populate_adjacent_cells(grid)
    print(grid)
    print(start_x,start_y)
    print(grid[start_x][start_y])
    print(end_x,end_y)
    print(grid[end_x][end_y])
    GRID = []
    for i in range(20):
        listt = []
        for j in range(20):
            listt.append(grid[i][j])
            GRID.append(listt)
    print(np.array(GRID)) 
    Path = a_star(start_node, goal_node, np.flip(grid).T)

    print("Path:", Path)
    if Path == None :
        print("Path could not be found")
        continue
    navigate(Path)
    print("Destination ",goal," reached.")