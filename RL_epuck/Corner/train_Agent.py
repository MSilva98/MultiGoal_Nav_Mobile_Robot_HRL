"""train_Agent controller."""
# python script to train the robot with a given RWD Table

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent

# print without scientific notation
np.set_printoptions(suppress=True)

class agentController():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()

        # Robot reinforcement learning brain in corner (currently training)
        self.brain = Agent(alpha=0.5, beta=0.05, ro=0, epsilon=0.3, sensors_states=6, RWDTable="RwdTable_v7_6s_with_corridor.txt")
        # self.brain = Agent(alpha=0.5, beta=0.05, ro=0, epsilon=0.2, sensors_states=6, RWDTable="RwdTable_v7_6states.txt")
        # Robot brain when in corridor (already trained)
        self.corridorBrain = Agent(epsilon=0, Qtable="../Corridor/QTable_v3_1cm.txt")

        # node to use supervisor functions
        self.robot_node = self.supervisor.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        self.initial_pos = self.translation_field.getSFVec3f()
        self.initial_rot = self.rotation_field.getSFRotation()

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
            self.ds.append(self.supervisor.getDistanceSensor(self.dsNames[i]))
            self.ds[i].enable(self.timestep)

        self.leftMotor = self.supervisor.getMotor('left wheel motor')
        self.rightMotor = self.supervisor.getMotor('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.run()

    def run(self):
        first = True
        end = False
        action = ''
        cur_state = 0
        next_state = 0
        corridor = True

        while self.supervisor.step(self.timestep) != -1:
            t = self.supervisor.getTime()
            while self.supervisor.getTime() - t < 0.05:
                # read sharp sensors outputs
                dsValues = []
                for i in range(len(self.ds)):
                    dsValues.append(self.ds[i].getValue())
                
                # controller termination
                if self.supervisor.step(self.timestep) == -1:
                    end = True
                    break

            next_state = self.brain.sensorsToState(dsValues, False) # next state - state after action
            # print(next_state)
            # break
            # Get distances of sensors voltages
            F  = self.brain.sensorVoltageToDistance(dsValues[0]) # Front
            FL = self.brain.sensorVoltageToDistance(dsValues[1]) # Front Left
            L  = self.brain.sensorVoltageToDistance(dsValues[2]) # Left
            FR = self.brain.sensorVoltageToDistance(dsValues[3]) # Front Right
            R  = self.brain.sensorVoltageToDistance(dsValues[4]) # Right
            B  = self.brain.sensorVoltageToDistance(dsValues[5]) # Back
            # cur_pos = [round(p,4) for p in self.translation_field.getSFVec3f()]
            # cur_ori = round(self.rotation_field.getSFRotation()[3],3)

            # print(round(F,3), round(FL,3), round(L,3), round(FR,3), round(R,3), round(B,3))
            # When in corridor just go
            # if L < 0.3 and R < 0.3 and (FR < 0.3 or FL < 0.3):
            # # if L < 0.3 and R < 0.3 and F == 0.3 and B == 0.3:
            # #     corridor = True

            # # if B < 0.3 or F < 0.3 or (L < 0.3 and R == 0.3 and FL < 0.3) or (L == 0.3 and R < 0.3 and FR < 0.3):
            # #     corridor = False

            # # if corridor:
            #     # New cicle starts here
            #     print("Corridor")
            #     cur_state = self.brain.sensorsToState(dsValues, True)   # next state - state after action
            #     action = self.corridorBrain.chooseAction(cur_state)     # best action in current state
            #     speeds = self.corridorBrain.actionToSpeed(action)       # speed of each motor
                
            # # When in Corner do the training
            # else:
            next_state = self.brain.sensorsToState(dsValues, False) # next state - state after action
            if not first:   # first step won't update table before take action and won't update if in same state
                # This happens after Take Action
                self.brain.updateQTable(cur_state, next_state, action)
                
            # New cicle starts here
            cur_state = next_state                      # current state - state before action
            action = self.brain.chooseAction(cur_state) # best action in current state
            speeds = self.brain.actionToSpeed(action)   # speed of each motor

            t = self.supervisor.getTime()
            while self.supervisor.getTime() - t < 0.1:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.supervisor.step(self.timestep) == -1:
                    end = True
                    break    

            # Reduce exploration probability after some time, first to 0.1 and then to 0.05
            # if t > 150000:
            #     if t > 300000:
            #         print("EPSILON: 0.05")
            #         self.brain.epsilon = 0.05
            #     else:
            #         print("EPSISLON: 0.1")
            #         self.brain.epsilon = 0.1
            # else:
            #     print("EPSILON: 0.2")
            
            first = False
            if end:
                break

        # Enter here exit cleanup code.
        print("SAVING TABLE...")
        self.brain.saveQTable("QTable_v7_6s_with_corridor.txt")
        print("TABLE SAVED!")
        exit(0)

agentController()