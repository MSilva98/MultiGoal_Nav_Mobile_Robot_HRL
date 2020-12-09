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

MAX_SPEED = 6.28

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

# enable all distance sensors
for i in range(len(psNames)):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(timestep)

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
    # Read the sensors:
    psValues = []
    for i in range(len(ps)):
        psValues.append(ps[i].getValue())

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
    right_obs = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obs = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    # initialize motor speeds at 50%
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    
    # modify speeds acording to obstacles
    if left_obs:
        # turn right
        leftSpeed += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
    elif right_obs:
        # turn left
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
        
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    

# Enter here exit cleanup code.
