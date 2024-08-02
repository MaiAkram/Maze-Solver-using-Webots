"""Maze_Left controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera
import cv2
import numpy as np

def color_detection(image):
    # Define color thresholds in RGB format
    red_lower = np.array([0, 0, 100], dtype=np.uint8)
    red_upper = np.array([50, 50, 255], dtype=np.uint8)

    # Threshold the image to get only desired colors
    red_mask = cv2.inRange(image, red_lower, red_upper)

    # Bitwise-AND mask and original image
    red_result = cv2.bitwise_and(image, image, mask=red_mask)

    # Normalize the color arrays to range from 0 to 1
    red_array = red_result.astype(np.float32) / 255.0
    
    # Check which color has the most pixels
    red_count = cv2.countNonZero(red_mask)
   
    # Determine the dominant color
    if red_count > 0.5:
        return [1, 0, 0]  # Red
    else:
        return "No dominant color detected"
        
def run_robot(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    max_speed = 6.28
    speeds = [max_speed, max_speed]
    
    motors = []
    left_motor = robot.getMotor('motor_1')
    right_motor = robot.getMotor('motor_2')
    motors.append(left_motor)
    motors.append(right_motor)
    for i in range(2):
        motors[i].setPosition(float('inf'))
        motors[i].setVelocity(0.0)
        
    sensors = []
    for i in range(5):
        sensor = robot.getDistanceSensor(f'ds{i}')
        sensors.append(sensor)
        sensors[i].enable(timestep)
    rf0 = robot.getRangeFinder(f'rf0')
    
    # Enable the camera
    cam = robot.getDevice("cam")
    cam.enable(timestep)

    n = 0;
    
    # Main loop:    
    while robot.step(timestep) != -1:
        # Read the sensors.
        for ind in range(5):
            print("ind: {}, val: {}".format(ind, sensors[ind].getValue()))
                
        # Process sensors data.
        front_wall = sensors[2].getValue() > 200
        front_wall_r = sensors[3].getValue() > 400
        front_wall_l = sensors[4].getValue() > 400
        left_wall = sensors[0].getValue() > 200
        right_wall = sensors[1].getValue() > 200
        
        # Get camera image
        imageI = cam.getImageArray()
        col = "None"
        
        if imageI:
            # Convert image to numpy array
            imageI = np.array(imageI, dtype=np.uint8)
            imageI = cv2.cvtColor(imageI, cv2.COLOR_RGB2BGR)
            
            # Perform color detection
            detected_color = color_detection(imageI)
            
            if (detected_color == [1, 0, 0]):
                col = "Red"
                print("GOAL END REACHED !")
                for i in range(2):
                    motors[i].setVelocity(0.0)
                break;
                
            print("Detected color:", col)
            
            # Display the image
            cv2.imshow("Camera Image", imageI)
            cv2.waitKey(1)  # Keep the window open until a key is pressed            

        
        if (front_wall or front_wall_l) or ((n>0) and (n<35)):
            front_wall = True
            n = n+1;
            print(n)
        elif n >= 35:
            n = 0
        
        if front_wall or front_wall_r or front_wall_l:
        #if front_wall:
            print("Turn Right in Place")
            speeds[0] = max_speed
            speeds[1] = -max_speed
            print(front_wall)
            
            """ps_values = [0, 0]
            dist_values = [0, 0]
            ps_values = [PS_2.getValue(), PS_1.getValue()]
            for ind in range(2):
                dist_values[ind] = ps_values[ind] * encoder_unit
                print("distance sensor values:", dist_values)
            theta = (dist_values[0]-dist_values[1])/wheel_base
            """
        else:
            if left_wall:
                print("Drive Forward")
                speeds[0] = max_speed
                speeds[1] = max_speed
            else:
                print("Turn Left")
                speeds[0] = max_speed/3.5
                speeds[1] = max_speed
        
        for i in range(2):
            motors[i].setVelocity(speeds[i])
            

if __name__ == "__main__":

    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)


