"""run_Agent controller."""
# python script to control the robot with a QTable after training

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent
import json

# print without scientific notation
np.set_printoptions(suppress=True)

class agentController():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.robot = Supervisor()

        self.robot_node = self.robot.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        # Robot reinforcement learning brain
        # NOTE: epsilon MUST be 0 to disable random actions
        self.frontBrain = Agent(epsilon=0, Qtable="QTable_corridor.txt")  # QTable for corridor and to go forward
        self.rightBrain = Agent(epsilon=0, Qtable="QTable_right_all.txt") # QTable to go right
        self.leftBrain  = Agent(epsilon=0, Qtable="QTable_left_all.txt")  # QTable to go left

        # get the time step of the current world.
        self.timestep = 32

        self.MAX_SPEED = 6.28

        # initialize devices
        self.ds = []
        self.dsNames = [
            'sharps0',  # front
            'sharps1',  # front left
            'sharps2',  # left
            'sharps3',  # front right
            'sharps4',  # right
            'sharps5'   # rear
        ]
        # enable sharp distance sensors
        for i in range(len(self.dsNames)):
            self.ds.append(self.robot.getDistanceSensor(self.dsNames[i]))
            self.ds[i].enable(self.timestep)

        self.leftMotor  = self.robot.getMotor('left wheel motor')
        self.rightMotor = self.robot.getMotor('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.run()

    def run(self):
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        end      = False
        action   = ''
        left     = False
        right    = False
        front    = False
        corridor = True

        while self.robot.step(self.timestep) != -1:    
            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.05:
                # read sharp sensors outputs
                dsValues = []
                for i in range(len(self.ds)):
                    dsValues.append(self.ds[i].getValue())
                
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break

            # Get distances of sensors' voltages
            F  = self.frontBrain.sensorVoltageToDistance(dsValues[0])  # Front
            FL = self.frontBrain.sensorVoltageToDistance(dsValues[1])  # Front Left
            L  = self.frontBrain.sensorVoltageToDistance(dsValues[2])  # Left
            FR = self.frontBrain.sensorVoltageToDistance(dsValues[3])  # Front Right
            R  = self.frontBrain.sensorVoltageToDistance(dsValues[4])  # Right
            B  = self.frontBrain.sensorVoltageToDistance(dsValues[5])  # Back

            # DETECT DOOR OR CORNER
            # Left Right Door
            if FR == 0.3 and FL == 0.3 and corridor:
                actDoor = np.random.choice(["left", "right"])
                if actDoor == "right":
                    print("ROBOT REACHED LR DOOR AND GOES RIGHT")
                    right = True
                    left  = False
                else:
                    print("ROBOT REACHED LR DOOR AND GOES LEFT")
                    right = False
                    left  = True
                front     = False
                corridor  = False
                
            # Front Right door/Corridor
            elif FR == 0.3 and R == 0.3 and B == 0.3:
                if corridor:
                    # Front Right Door
                    if F == 0.3:
                        actDoor = np.random.choice(["front", "right"])
                        if actDoor == "right":
                            print("ROBOT REACHED FR DOOR AND GOES RIGHT")
                            right = True
                            front = False
                        elif actDoor == "front":
                            print("ROBOT REACHED FR DOOR AND GOES FORWARD")
                            front = True
                            right = False
                    # Corner to Right
                    else:
                        right = True
                        front = False
                    corridor  = False

            # Front Left door/Corridor
            elif FL == 0.3 and L == 0.3 and B == 0.3:
                if corridor:
                    # Front Left Door
                    if F == 0.3:
                        actDoor = np.random.choice(["front", "left"])
                        if actDoor == "left":
                            print("ROBOT REACHED FL DOOR AND GOES LEFT")
                            left  = True
                            front = False
                        elif actDoor == "front":
                            print("ROBOT REACHED FL DOOR AND GOES FORWARD")
                            front = True
                            left  = False
                    # Left Corner
                    else:
                        left  = True
                        front = False
                    corridor  = False    
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R <= 0.2 and L <= 0.2 and R < FR and L < FL and B == 0.3 and F == 0.3 and (left or right or front):
                print("ROBOT EXITS DOOR/CORNER AND IS ON CORRIDOR")
                front    = False
                left     = False
                right    = False
                corridor = True

            # Current state of the robot
            state = self.frontBrain.sensorsToState(dsValues)        
            if left and not right and not front and not corridor:
                print("Turn Left")
                action = self.leftBrain.chooseAction(state) # best action in current state 
            elif right and not left and not front and not corridor:
                print("Turn Right")
                action = self.rightBrain.chooseAction(state)
            elif (front or corridor) and not right and not left:
                if front:
                    print("Go Forward")
                action = self.frontBrain.chooseAction(state)
            else:
                print("ONLY ONE SHOULD BE TRUE F:", front, " R:", right, " L:", left, " C:", corridor)
                end = True
                break
            speeds = self.frontBrain.actionToSpeed(action)   # speed of each motor
            print("State: ", state, "Action: ", action)

            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.1:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break    
            if end:
                break

        # Enter here exit cleanup code.
        exit(0)

agentController()