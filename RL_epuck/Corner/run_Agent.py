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
        # self.robot = Robot()

        self.robot = Supervisor()
        self.robot_node = self.robot.getFromDef("epuck")
        self.rotation_field = self.robot_node.getField("rotation")
        self.translation_field = self.robot_node.getField("translation")


        # Robot reinforcement learning brain
        # self.cornerRight = Agent(alpha=0, beta=0, ro=0, epsilon=0, sensors_states=6, Qtable="QTable_corridor_v5_6500h_right.txt")
        # self.cornerLeft = Agent(alpha=0, beta=0, ro=0, epsilon=0, sensors_states=6, Qtable="QTable_corridor_v5_6500h_left_symmetric.txt")
        # self.cornerLeft = Agent(alpha=0, beta=0, ro=0, epsilon=0, sensors_states=6, Qtable="QTable_corridor_v5_6500h_left.txt")
        # self.cornerBrain = Agent(alpha=0, beta=0, ro=0, epsilon=0, Qtable="QTable_corridor_transform.txt")
        # self.corridorBrain = Agent(alpha=0, beta=0, ro=0, epsilon=0, Qtable="../Corridor/QTable_v3_1cm.txt")

        self.cornerRight = json.load(open("QTable_corridor_v5_6500h_right.txt"))
        self.cornerLeft = json.load(open("QTable_corridor_v5_6500h_left_symmetric.txt"))
        self.agent = Agent(epsilon=0)

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
        leftCorner = False
        rot_vals = [x*math.pi/180 for x in range(-180,181)]
        # rot_vals = [-2]

        while self.robot.step(self.timestep) != -1:    
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]

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
                
            # Current state of the robot
            # state = self.cornerRight.sensorsToState(dsValues, False)        
            state = self.agent.sensorsToState(dsValues, False)        

            # Get distances of sensors voltages
            # F  = self.cornerRight.sensorVoltageToDistance(dsValues[0])  # Front
            # FL = self.cornerRight.sensorVoltageToDistance(dsValues[1])  # Front Left
            # L  = self.cornerRight.sensorVoltageToDistance(dsValues[2])  # Left
            # FR = self.cornerRight.sensorVoltageToDistance(dsValues[3])  # Front Right
            # R  = self.cornerRight.sensorVoltageToDistance(dsValues[4])  # Right
            # B  = self.cornerRight.sensorVoltageToDistance(dsValues[5])  # Back

            F  = self.agent.sensorVoltageToDistance(dsValues[0])  # Front
            FL = self.agent.sensorVoltageToDistance(dsValues[1])  # Front Left
            L  = self.agent.sensorVoltageToDistance(dsValues[2])  # Left
            FR = self.agent.sensorVoltageToDistance(dsValues[3])  # Front Right
            R  = self.agent.sensorVoltageToDistance(dsValues[4])  # Right
            B  = self.agent.sensorVoltageToDistance(dsValues[5])  # Back
            

            # Corridor or right corner
            if L < 0.2 and R < 0.2 and F == 0.3 and B == 0.3 and FR > R:
                if FL == 0.3:
                    leftCorner = True
                else:
                    leftCorner = False
                
            if leftCorner:
                print(F, FL, L, FR, R, B)
                print(state, leftCorner, cur_pos[0], cur_pos[2])

            if leftCorner:
                print("Left")
                # action = self.cornerLeft.chooseAction(state) # best action in current state
                action = self.cornerLeft[state] # best action in current state
            # When in Corner do the training
            else:        
                print("Right")
                # action = self.cornerRight.chooseAction(state) # best action in current state
                action = self.cornerRight[state] # best action in current state
            
            # action = self.cornerRight.chooseAction(state) # best action in current state

            # speeds = self.cornerRight.actionToSpeed(action)   # speed of each motor
            speeds = self.agent.actionToSpeed(action)   # speed of each motor
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