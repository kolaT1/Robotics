import math
from spike import PrimeHub, LightMatrix, MotorPair, Motor, MotionSensor, ColorSensor, ForceSensor, PIDController
from spike.control import Timer, wait_for_seconds
# Import the PIDController class
from spike.control import PIDController
	
# ------------------- SpikeRescue -------------
motor_pair = MotorPair('C', 'D')
motorLeft = Motor('C')
motorRight = Motor('D')
armMotor = Motor('B')
# Instantiate the sensors with the correct port names
motion_sensor = MotionSensor()
color_sensor = ColorSensor('E')
force_sensor = ForceSensor('F')
hub = PrimeHub()
timer = Timer()
timer.reset()
lm = LightMatrix()
# --------- Frontier 1 -------------------------
# Define a function to calculate the distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
# Define a function to calculate the next point to move to
# using the Frontier Algorithm
def next_point(current_x, current_y, destination_x, destination_y):
    # Calculate the distance to the destination
    d = distance(current_x, current_y, destination_x, destination_y)
    # If we are within a certain distance of the destination, stop
    if d < 5:

        return current_x, current_y
    # Otherwise, calculate the next point to move to using the Frontier Algorithm
    else:
        # Calculate the angle to the destination
        theta = math.atan2(destination_y - current_y, destination_x - current_x)
        # Calculate the next point to move to, based on the angle and a fixed step size
        next_x = current_x + math.cos(theta) * 10
        next_y = current_y + math.sin(theta) * 10
        return next_x, next_y
# Define the coordinates of the starting point and the destination
start_x = 0
start_y = 0
destination_x = 100
destination_y = 100
# Initialize the current position to the starting point
current_x = start_x
current_y = start_y
# Create a PIDController 
pid = PIDController(kp=1.0, ki=0.1, kd=0.5)
# Set the target position for the PID controller
pid.set_target(destination_x, destination_y)

# Keep moving until we reach the destination
while True:

    # Update the PID controller and get the control output
    output = pid.update(current_x, current_y)

    # Use the control output to set the motor speeds
    motorLeft.spin(output[0])
    motorRight.spin(output[1])

    # Update the current position using the motion sensor
    current_x, current_y = motion_sensor.position()

    # If we have reached the destination, stop
    if current_x == destination_x and current_y == destination_y:
        break

# -------------- Frontier 2 -------------------
# Define the robot's starting position and orientation
start = (0, 0)
orientation = 0
# Define the dimensions of the maze
width = 10
height = 10
# Define the maze as a 2D grid
grid = [[0 for x in range(width)] for y in range(height)]
# Set the robot's starting position on the grid
grid[start[0]][start[1]] = 1
# Define the movements that the robot can make
# (up, right, down, left)
movements = [(0, 1), (1, 0), (0, -1), (-1, 0)]
# Define the Frontier Algorithm function
def frontier(grid, start):
    # Initialize the queue with the starting position
    queue = [start]
    # Initialize the visited list with the starting position
    visited = [start]
    # Loop until the queue is empty
    while len(queue) > 0:
        # Get the current position from the queue
        current = queue.pop(0)
        # Loop through the possible movements
        for move in movements:
            # Calculate the new position
            new_pos = (current[0] + move[0], current[1] + move[1])
            # Check if the new position is valid and not already visited
            if new_pos[0] >= 0 and new_pos[0] < width and new_pos[1] >= 0 and new_pos[1] < height and grid[new_pos[0]][new_pos[1]] != 1:
                # Add the new position to the queue
                queue.append(new_pos)
                # Add the new position to the visited list
                visited.append(new_pos)
                # Set the new position on the grid
                grid[new_pos[0]][new_pos[1]] = 1
    # Return the visited list
    return visited
# Call the Frontier Algorithm function to get the visited positions
visited = frontier(grid, start)
# Loop through the visited positions
for pos in visited:
    # Calculate the movement to get to the next position
    move = (pos[0] - start[0], pos[1] - start[1])
    # Update the robot's orientation
    orientation = (orientation + movements.index(move)) % 4
    # Move the robot to the next position
    robot.drive_distance(1, orientation * 90)
    # Update the robot's starting position
    start = pos
    	
# ------------------- SLAM ----------------------------

# Create a Timer object to periodically update the robot's position and orientation
timer = Timer(period=0.1, target_fps=None)

# Initialize the robot's position and orientation
x = 0
y = 0
theta = 0

while True:
    # Wait for the next update interval
    wait_for_seconds(timer.update_time_remaining())

    # Update the robot's position and orientation using the data from the MotionSensor and ColorSensor objects
    x += motion_sensor.get_displacement() * math.cos(theta)
    y += motion_sensor.get_displacement() * math.sin(theta)
    theta += motion_sensor.get_rotation()

    # Use the ForceSensor object to detect when the robot collides with an obstacle
    # and adjust its path accordingly
    if force_sensor.is_pushed():
        # Stop the robot and adjust its orientation
        motor_pair.stop()
        theta += math.pi / 2

        # Move the robot in a new direction
        motor_pair.drive_distance(1, theta)

# ------------------ SLAM 2 ---------------------------------------------

    # move the robot forward for 1 second
    motor_left.run_angle(speed=100, angle=360, then=Motor.STOP, wait=False)
    motor_right.run_angle(speed=100, angle=360, then=Motor.STOP, wait=True)

    # wait for 1 second
    wait_for_seconds(1)

    # perform SLAM
    # initialize map and current position
    map = []
    current_position = [0, 0]

    # loop until the robot reaches its destination
    while not reached_destination:
    # read distance from ultrasonic sensor
    distance = ultrasonic_sensor.distance_centimeters()

    # update map and current position using SLAM algorithm
    # TODO: implement SLAM algorithm

    # initialize state and covariance matrices
    state = np.array([[current_position[0]], [current_position[1]], [0], [0]])
    covariance = np.eye(4)

    # process noise matrix
    Q = np.array([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]])

    # measurement noise matrix
    R = np.array([[0.1, 0], [0, 0.1]])

    # measurement matrix
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    # loop until the robot reaches its destination
    while not reached_destination:
    # read distance from ultrasonic sensor
    distance = ultrasonic_sensor.distance_centimeters()

    # predict state and covariance
    state = np.array([[state[0][0] + state[2][0]], [state[1][0] + state[3][0]], [state[2][0]], [state[3][0]]])
    covariance = covariance + Q

    # update state and covariance using Kalman filter
    k

    # move the robot forward
    motor_left.run_angle(speed=100, angle=360, then=Motor.STOP, wait=False)
    motor_right.run_angle(speed=100, angle=360, then=Motor.STOP, wait=True)

    # stop the robot
    motor_left.stop()
    motor_right.stop()
