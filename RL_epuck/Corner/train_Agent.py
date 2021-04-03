"""fill_QTable controller."""
# python script to collect data from Sharp and infrared distance sensors

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import *

# print without scientific notation
np.set_printoptions(suppress=True)

class agentController():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()

        # Robot reinforcement learning brain
        self.brain = Agent(alpha=0.5, beta=0.05, ro=0, epsilon=0.2, RWDTable="RwdTable_newRWD.txt")

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
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        first = True
        end = False
        action = ''
        cur_state = 0
        next_state = 0

        while 1:
            t = self.supervisor.getTime()
            while self.supervisor.getTime() - t < 0.01:
                # read sharp sensors outputs
                dsValues = []
                for i in range(len(self.ds)):
                    dsValues.append(self.ds[i].getValue())
                
                # controller termination
                if self.supervisor.step(self.timestep) == -1:
                    end = True
                    break

            next_state = self.brain.sensorsToState(dsValues) # next state - state after action

            if not first:   # first step won't update table before take action
                # This happens after Take Action
                self.brain.updateQTable_SuttonBarto(cur_state, next_state, action)
                    
            # New cicle starts here
            cur_state = next_state                      # current state - state before action
            action = self.brain.chooseAction(cur_state) # best action in current state
            speeds = self.brain.actionToSpeed(action)   # speed of each motor
            
            t = self.supervisor.getTime()
            while self.supervisor.getTime() - t < 0.5:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.supervisor.step(self.timestep) == -1:
                    end = True
                    break    

            # Reduce exploration probability after some time, firt to 0.1 and then to 0.05
            if t > 9000:
                if t > 18000:
                    print("EPSILON: 0.05")
                    self.brain.epsilon = 0.05
                else:
                    print("EPSISLON: 0.1")
                    self.brain.epsilon = 0.1
            else:
                print("EPSILON: 0.2")
            
            first = False
            if end:
                break

        # Enter here exit cleanup code.
        self.brain.saveQTable()
        exit(0)

agentController()