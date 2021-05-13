"""states_positions controller."""
# python script to get all possible states in FLR door for each X,Z,theta

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent
import json

# print without scientific notation
np.set_printoptions(suppress=True)

class states_positions():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()

        # Robot reinforcement learning brain
        self.brain = Agent()

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

        self.run()
        
    def run(self):
        # Values from left to right
        x_vals = [x/100 for x in range(-11, 12)]  # corner to right
        # Values from ip to down
        z_vals = [z/100 for z in range(-11, 12)]  # -96 to -74 is Corner
        # 360 degrees in radians
        rot_vals = [x*math.pi/180 for x in range(0,360,5)]
        # rot_vals = [-90*math.pi/180]

        for rot in rot_vals:                    
            # Set robot rotation
            self.rotation_field.setSFRotation([0,1,0,rot])
            states = dict()
            for x in x_vals:
                for z in z_vals:
                    # Set robot position
                    self.translation_field.setSFVec3f([x,0,z])
                    self.robot_node.resetPhysics()
                    t = self.supervisor.getTime()
                    # 0.05 guarantees a read of actual values of the sensors
                    while self.supervisor.getTime() - t < 0.05:
                        # read sharp sensors outputs
                        dsValues = []
                        for i in range(len(self.ds)):
                            dsValues.append(self.ds[i].getValue())
                    
                        # controller termination
                        if self.supervisor.step(self.timestep) == -1:
                            quit()
                        
                    cur_x = x
                    cur_z = z
                    print(cur_x, cur_z, rot)

                    state = self.brain.sensorsToState(dsValues, False)
                    if state not in states.keys():
                        states[state] = [(cur_x, cur_z, rot)]
                    else:
                        states[state].append((cur_x, cur_z, rot))
            
            # Enter here exit cleanup code.
            with open("state_maps/FR/FR_6_states_"+str(round(rot*180/math.pi,0))+".txt", "w+") as jsonFile:
                json.dump(states, jsonFile)

        exit(0)

states_positions()