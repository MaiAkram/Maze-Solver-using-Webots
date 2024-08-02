"""Maze_Left controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

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
    for i in range(8):
        sensor = robot.getDistanceSensor(f'ds{i}')
        sensors.append(sensor)
        sensors[i].enable(timestep)
    rf0 = robot.getRangeFinder(f'rf0')
    
    """
    # compute encoder unit
    wheel_radius = 0.05
    wheel_base = 0.3
    wheel_circumference = 2 * 3.14 * wheel_radius
    encoder_unit = wheel_circumference / 6.28
    theta = 3.14
    """
    n = 0;
    
    # Main loop:    
    while robot.step(timestep) != -1:
        # Read the sensors.
        for ind in range(8):
            print("ind: {}, val: {}".format(ind, sensors[ind].getValue()))
                
        # Process sensors data.
        front_wall = sensors[2].getValue() > 200
        front_wall_r = sensors[7].getValue() > 400
        front_wall_l = sensors[6].getValue() > 400
        left_wall = sensors[0].getValue() > 200
        right_wall = sensors[1].getValue() > 200
        
        """
        if theta < 1.57:
            front_wall = 1
        elif theta == 1.57:
            theta = 3.14
        """
        
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


