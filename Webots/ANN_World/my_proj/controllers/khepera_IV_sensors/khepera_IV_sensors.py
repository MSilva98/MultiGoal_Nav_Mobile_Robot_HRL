"""epuck_sensors_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import matplotlib.pyplot as plt

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
timestep = 64

MAX_SPEED = 47.6

# initialize devices
us = []
usNames = [
    'left ultrasonic sensor',
    'front left ultrasonic sensor',
    'front ultrasonic sensor',
    'right ultrasonic sensor',
    'front right ultrasonic sensor'
]

# enable ultra-sonic distance sensors
for i in range(len(usNames)):
    us.append(robot.getDistanceSensor(usNames[i]))
    us[i].enable(timestep)
    
ir = []
irNames = [
    'rear left infrared sensor',
    'rear infrared sensor',
    'rear right infrared sensor',
    'left infrared sensor',
    'right infrared sensor',
    'front left infrared sensor',
    'front right infrared sensor',
    'front infrared sensor',
    'ground left infrared sensor',
    'ground front left infrared sensor',
    'ground front right infrared sensor',
    'ground right infrared sensor',
]
# enable ultra-sonic distance sensors
for i in range(len(irNames)):
    ir.append(robot.getDistanceSensor(irNames[i]))
    ir[i].enable(timestep)
    

# camera    
camera = robot.getCamera('camera')
camera.enable(timestep)

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # get image from camera
    image = camera.getImageArray()
    # get RGB value from each pixel of image
    #for x in range(0, camera.getWidth()):
    #    for y in range(0, camera.getHeight()):
    #        r = image[x][y][0]
    #        g = image[x][y][1]
    #        b = image[x][y][2]
    #        print("r = " + str(r) + " g = " + str(g) + " b = " + str(b))
    
    camera.saveImage("test.jpeg", 100)
    
    # Process sensor data here.
    speed_offset = 0.2 * (MAX_SPEED - 0.03 * ir[7].getValue())
    speed_delta = 0.03 * ir[5].getValue() - 0.03 * ir[6].getValue()
    
    leftMotor.setVelocity(speed_offset + speed_delta)
    rightMotor.setVelocity(speed_offset - speed_delta)
    

# Enter here exit cleanup code.
