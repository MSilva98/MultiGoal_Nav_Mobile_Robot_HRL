"""test_180degrees_action controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 32

leftMotor  = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
axelLength = 0.052
wheelRadius = 0.0205
wheelSpeed = 2
angle = 3.14
delta = angle*axelLength/(2*wheelRadius*wheelSpeed)

t = robot.getTime()
while robot.getTime() - t <= delta:
    # Take action
    leftMotor.setVelocity(wheelSpeed)
    rightMotor.setVelocity(-wheelSpeed)
    # controller termination
    if robot.step(timestep) == -1:
        end = True
        break 


# Enter here exit cleanup code.