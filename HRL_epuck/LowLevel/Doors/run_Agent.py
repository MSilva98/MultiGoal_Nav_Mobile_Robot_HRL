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
        # USING RAW QTables
        # NOTE: epsilon MUST be 0 to disable random actions
        # self.frontBrain = Agent(epsilon=0, Qtable="./v4/QTable_v5_13000h_Front_all.txt") # QTable for corridor and to go forward
        # self.leftBrain  = Agent(epsilon=0, Qtable="./v4/QTable_v5_13000h_FL_LR_Left.txt")   # QTable to go left on FL doors
        # self.rightBrain = Agent(epsilon=0, Qtable="./v4/QTable_v5_13000h_FR_LR_Right.txt")  # QTable to go right on FR doors
        
        # self.frontBrain = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_FR_Front.txt") # QTable for corridor and to go forward
        # self.leftBrain  = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_FR_FL.txt")   # QTable to go left on FL doors
        # self.rightBrain = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_FR_FL.txt")  # QTable to go right on FR doors

        self.frontBrain = Agent(epsilon=0) # QTable for corridor and to go forward
        self.leftBrain  = Agent(epsilon=0)   # QTable to go left on FL doors
        self.rightBrain = Agent(epsilon=0)  # QTable to go right on FR doors

        # # Using trimmed QTables: each state just has the best action
        # self.frontBrain = json.load(open("./v4/QTable_v5_13000h_FR_Front_all_TRIMMED.txt"))   # QTable for corridor and to go forward
        # self.leftBrain  = json.load(open("./v4/QTable_v5_5000h_FL_Left_z21_TRIMMED.txt"))   # QTable to go left on doors
        # self.rightBrain = json.load(open("./v4/QTable_v5_5000h_FR_Right_Symmetry_z21_TRIMMED.txt")) # QTable to go right on doors
        # self.agent      = Agent(epsilon=0) 


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
        end    = False
        action = ''
        left   = False
        right  = False
        front  = False
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
            # USING RAW TABLES
            F  = self.frontBrain.sensorVoltageToDistance(dsValues[0])  # Front
            FL = self.frontBrain.sensorVoltageToDistance(dsValues[1])  # Front Left
            L  = self.frontBrain.sensorVoltageToDistance(dsValues[2])  # Left
            FR = self.frontBrain.sensorVoltageToDistance(dsValues[3])  # Front Right
            R  = self.frontBrain.sensorVoltageToDistance(dsValues[4])  # Right
            B  = self.frontBrain.sensorVoltageToDistance(dsValues[5])  # Back

            # # USING TRIMMED TABLES
            # F  = self.agent.sensorVoltageToDistance(dsValues[0])  # Front
            # FL = self.agent.sensorVoltageToDistance(dsValues[1])  # Front Left
            # L  = self.agent.sensorVoltageToDistance(dsValues[2])  # Left
            # FR = self.agent.sensorVoltageToDistance(dsValues[3])  # Front Right
            # R  = self.agent.sensorVoltageToDistance(dsValues[4])  # Right
            # B  = self.agent.sensorVoltageToDistance(dsValues[5])  # Back

            # DETECT DOOR OR CORNER
            # Left Right Door
            if FR == 0.3 and FL == 0.3 and L == 0.3 and R == 0.3 and corridor:
                actDoor = np.random.choice(["left", "right"])
                if actDoor == "right":
                    print("ROBOT REACHED LR DOOR AND GOES RIGHT", F, FL, L, FR, R, B)
                    right = True
                    left  = False
                else:
                    print("ROBOT REACHED LR DOOR AND GOES LEFT", F, FL, L, FR, R, B)
                    right = False
                    left  = True
                front = False
                corridor = False

            # Front Right door/Corridor
            elif FR == 0.3 and R == 0.3:
                if corridor:
                    # Front Right Door
                    if F == 0.3:
                        actDoor = np.random.choice(["front", "right"])
                        if actDoor == "right":
                            print("ROBOT REACHED FR DOOR AND GOES RIGHT", F, FL, L, FR, R, B)
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
                    corridor = False
                    left = False

            # Front Left door/Corridor
            elif FL == 0.3 and L == 0.3:
                if corridor:
                    # Front Left Door
                    if F == 0.3:
                        actDoor = np.random.choice(["front", "left"])
                        if actDoor == "left":
                            print("ROBOT REACHED FL DOOR AND GOES LEFT", F, FL, L, FR, R, B)
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
                    corridor = False    
                    right = False
                
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R <= 0.2 and L <= 0.2 and B == 0.3 and F == 0.3 and (left or right or front):
                print("ROBOT EXITS DOOR/CORNER AND IS ON CORRIDOR")
                front = False
                left  = False
                right = False
                corridor = True

            # USING RAW QTABLES
            # Current state of the robot
            state = self.frontBrain.sensorsToState(dsValues, False)   
            print(state)
            break

            if left and not right and not front:
                print("LEFT DOOR")
                action = self.leftBrain.chooseAction(state) # best action in current state
            elif right and not left and not front:
                print("RIGHT DOOR")
                action = self.rightBrain.chooseAction(state)
            elif (front or corridor) and not left and not right:
                action = self.frontBrain.chooseAction(state)
            else:
                print("ONLY ONE SHOULD BE TRUE F:", front, " R:", right, " L:", left, " C:", corridor)
                end = True
                break
            speeds = self.frontBrain.actionToSpeed(action)   # speed of each motor

            # # USING TRIMMED QTABLES
            # state = self.agent.sensorsToState(dsValues, False) # next state - state after action                                
            # if left and not right and not front:
            #     action = self.leftBrain[state]
            # elif right and not left and not front:
            #     action = self.rightBrain[state]
            # elif front and not left and not right:
            #     action = self.frontBrain[state]
            # else:
            #     print("ONLY ONE SHOULD BE TRUE F:", front, " R:", right, " L:", left)
            #     end = True
            #     break
            # speeds = self.agent.actionToSpeed(action)   # speed of each motor
            
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