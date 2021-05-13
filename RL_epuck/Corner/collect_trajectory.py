"""collect_trajectory controller."""
# python script to obtain the robot trajectory going through doors

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
        # create supervisor instance (Set of fucorrected_v5_6s_4e_5000h_oncenctions available for each robot node)
        self.robot = Supervisor()
        # node to use Supervisor functions
        self.robot_node = self.robot.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")
        
        # Robot reinforcement learning brain
        # NOTE: epsilon MUST be 0 to disable random actions
        # self.leftCorner  = Agent(epsilon=0, Qtable="QTable_corner_left_sym_corrected.txt")  # QTable to go left on corner
        # self.rightCorner = Agent(epsilon=0, Qtable="QTable_corner_right_corrected.txt")     # QTable to go right on corner

        self.leftCorner  = Agent(epsilon=0, Qtable="QTable_corner_v5_17500h_left_sym.txt")  # QTable to go left on corner
        self.rightCorner = Agent(epsilon=0, Qtable="QTable_corner_v5_17500h_right.txt")     # QTable to go right on corner

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

        self.leftMotor = self.robot.getMotor('left wheel motor')
        self.rightMotor = self.robot.getMotor('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.run()

    def run(self):
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        end = False
        action = ''
        all_pos_right = []
        all_pos_left = []
        last_t = 0
        right = True
        leftCorner = False


        while self.robot.step(self.timestep) != -1:    
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]

            t = self.robot.getTime()
            # Everytime the robot reaches the corridor after the corner its position is reseted
            if cur_pos[0] > -0.40 and last_t < 3600:
                all_pos_right.append((0,0)) # "Point" just to separate each path
                right = True
                self.translation_field.setSFVec3f([-0.85,0,-0.4])
                self.rotation_field.setSFRotation([0,1,0,0])
                self.robot_node.resetPhysics()
            elif (cur_pos[2] > -0.40 or right) and last_t >= 3600:
                all_pos_left.append((0,0)) # "Point" just to separate each path
                right = False
                self.translation_field.setSFVec3f([-0.4,0,-0.85])
                self.rotation_field.setSFRotation([0,1,0,1.57])
                self.robot_node.resetPhysics()
                
            elif round(t,0) - last_t >= 1:
                last_t = round(t,0)
                if right:
                    all_pos_right.append((cur_pos[0], cur_pos[2]))
                else:
                    all_pos_left.append((cur_pos[0], cur_pos[2]))

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
            F  = self.rightCorner.sensorVoltageToDistance(dsValues[0])  # Front
            FL = self.rightCorner.sensorVoltageToDistance(dsValues[1])  # Front Left
            L  = self.rightCorner.sensorVoltageToDistance(dsValues[2])  # Left
            FR = self.rightCorner.sensorVoltageToDistance(dsValues[3])  # Front Right
            R  = self.rightCorner.sensorVoltageToDistance(dsValues[4])  # Right
            B  = self.rightCorner.sensorVoltageToDistance(dsValues[5])  # Back
            
            state = self.rightCorner.sensorsToState(dsValues, False)  

            # Corridor or right corner
            if L < 0.2 and R < 0.2 and F == 0.3 and B == 0.3 and FR > R:
                if FL == 0.3:
                    leftCorner = True
                else:
                    leftCorner = False
 
            if leftCorner:
                print("Left")
                action = self.leftCorner.chooseAction(state) # best action in current state
            else:        
                print("Right")
                action = self.rightCorner.chooseAction(state) # best action in current state
            speeds = self.rightCorner.actionToSpeed(action)   # speed of each motor

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

            # End or 2h passed
            if end or round(t,0) >= 7200:
                break
        
        print("SAVING TABLES...")
        with open("corner_right_17500.txt", "w+") as f:
            for p in all_pos_right:
                f.write(str(p) + "\n")
        print("DOOR RIGHT SAVED!")

        with open("corner_left_17500.txt", "w+") as f:
            for p in all_pos_left:
                f.write(str(p) + "\n")
        print("DOOR LEFT SAVED!")

        # Enter here exit cleanup code.
        exit(0)

agentController()