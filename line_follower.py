import numpy as np 
import pybullet as p 
import time
import math

p_id = p.connect(p.GUI)                #Loading the simulation
p.setGravity(0, 0, -10)                #Setting the gravity

plane = p.loadURDF("/home/shivam/catkin_ws/src/example/src/plane.urdf")        #Loading the plane
carPos = [0, 3, 0.1]                      #This is where the car will spawn, this is constant. Don't change

m = 0                           #Declaring the slope of the required line y = mx + c
c = 0                           #Declaring the constant of the required line  y = mx + c
angle = math.atan(m)

car = p.loadURDF("src/car/car1.urdf", carPos, p.getQuaternionFromEuler([0, 0, angle]))  #Loading the car with head parallel to the given line


def printLine(m, c):                        #This functions draws a line that we need to follow
    angle = math.atan(m)
    z = 0.02
    origin = [0, c, z]
    line = p.loadURDF("/home/shivam/catkin_ws/src/example/src/line.urdf", origin, p.getQuaternionFromEuler([0, 0, angle]))

printLine(m, c)                    #Calling the function to print the line


num = p.getNumJoints(car)                  #Getting the total number of joints in the car
for i in range(num):
    print(p.getJointInfo(car, i))           #Printing the information of each joint to get the motor joints


#These are the 4 motor joints that we need to manipulate, we declare them here.

fl = 2        #Front Left wheel        
fr = 3        #Front Right wheel
bl = 4        #Back Left wheel
br = 5        #Back Right wheel

p.setJointMotorControlArray(car, [fl, bl, fr, br], p.VELOCITY_CONTROL, forces=[0, 0, 0, 0])   #This is done to enable torque control in wheels of the car
p.stepSimulation()


#Declare the desired_state and base_torque globally
desired_state = 0  # Desired y-position (line y=0)
base_torque = 10 # Base torque

# PID Gains
kp = 1
ki = 0.0
kd = 0.01

# Initialize the error terms for PID
previous_error = 0
integral_error = 0

def moveCar(base_torque, action):
    """
    Control the car's motors using the calculated base torque and action.
    """
    # Apply the torques to the left and right motors
    p.setJointMotorControl2(bodyUniqueId=car, jointIndex=fl, controlMode=p.TORQUE_CONTROL, force=base_torque - action)
    p.setJointMotorControl2(bodyUniqueId=car, jointIndex=bl, controlMode=p.TORQUE_CONTROL, force=base_torque - action)
    p.setJointMotorControl2(bodyUniqueId=car, jointIndex=fr, controlMode=p.TORQUE_CONTROL, force=base_torque + action)
    p.setJointMotorControl2(bodyUniqueId=car, jointIndex=br, controlMode=p.TORQUE_CONTROL, force=base_torque + action)

def calc_error():
    """
    Calculate the error and required action using PID control.
    """
    global previous_error, integral_error

    # Get the car's current position and orientation
    position, orientation = p.getBasePositionAndOrientation(car)
    car_y = position[1]

    # Calculate the error based on the desired state
    error = desired_state - car_y

    # Calculate the derivative and integral of the error
    derivative_error = error - previous_error
    integral_error += error

    # PID control action
    action = kp * error + ki * integral_error + kd * derivative_error

    # Update previous error
    previous_error = error

    return action


# Select the simulation window and Press ENTER to execute

while True:                         # This while loop will run until ESCAPE key is pressed, then it will start the simulation.
    keycode = p.getKeyboardEvents()       # Getting the keyboard events through PyBullet
    if keycode.get(p.B3G_RETURN) == 1:                     # As soon as any key is pressed and it's ENTER key, simulation starts
        p.resetSimulation()       # Simulation is reset
        p.setGravity(0, 0, -10)

        plane = p.loadURDF("/home/shivam/catkin_ws/src/example/src/plane.urdf")
        car = p.loadURDF("src/car/car1.urdf", carPos, p.getQuaternionFromEuler([0, 0, angle]))   # Plane and car loaded again
        p.setJointMotorControlArray(car, [fl, bl, fr, br], p.VELOCITY_CONTROL, forces=[0, 0, 0, 0])   # This is done to enable torque control in wheels of the car
        printLine(m, c)   # This draws a line along y = 0, which we have to follow

        while True:
            p.resetDebugVisualizerCamera(7, -90, -45, p.getBasePositionAndOrientation(car)[0])  # This will keep the camera on the car always
            p.stepSimulation()    # This steps the simulation further by 0.01 seconds approx

            action = calc_error()
            moveCar(base_torque, action)

            time.sleep(1./240.)

            keycode = p.getKeyboardEvents()    # This will keep tracking if ENTER key is pressed again.
            if keycode.get(p.B3G_RETURN) == 1:              # We end the current simulation and start a new one again if ENTER key is pressed
                print("Episode finished")                   # This is a way to re-run the simulation without re-executing the code
                p.resetSimulation()  # Reseting the simulation
                break                # Breaking out of the inner while loop
