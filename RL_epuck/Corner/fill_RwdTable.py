"""fill_RwdTable controller."""
# python script to fill reward Table for corner in QLearning_square_maze.wbt

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent

# print without scientific notation
np.set_printoptions(suppress=True)

class fill_RwdTable():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()

        # Robot reinforcement learning brain
        self.brain = Agent(sensors_states=6)

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
        # Initial state -> robot aligned in center of corridor
        prev_state = "(4, 3, 2, 3, 2, 4)"
        # Values from left to right
        x_vals = [x/100 for x in range(-96, -73)]  # corner to right
        # Values from ip to down
        z_vals = [z/100 for z in range(-96, -45)]  # -96 to -74 is Corner
        # 360 degrees in radians
        rot_vals = [x*math.pi/180 for x in range(90,-91,-1)]

        for x in x_vals:
            for z in z_vals:
                # Set robot position
                self.translation_field.setSFVec3f([x,0,z])
                for rot in rot_vals:
                    # Set robot rotation
                    self.rotation_field.setSFRotation([0,1,0,rot])
                    self.robot_node.resetPhysics()

                    t = self.supervisor.getTime()
                    while self.supervisor.getTime() - t < 0.05:
                        # read sharp sensors outputs
                        dsValues = []
                        for i in range(len(self.ds)):
                            dsValues.append(self.ds[i].getValue())

                        # controller termination
                        if self.supervisor.step(self.timestep) == -1:
                            quit()
                        
                    cur_pos = [round(p,4) for p in self.translation_field.getSFVec3f()]
                    cur_ori = round(self.rotation_field.getSFRotation()[3],3)
                    
                    cur_pos[0] = round(cur_pos[0]+0.85,3) 
                    cur_pos[2] = round(cur_pos[2]+0.85,3)     

                    # add initial reward to that state in QTable
                    # dsValues = [front, front left, left, front right, right, rear]
                    prev_state = self.brain.fillRwdTable(dsValues, cur_pos, cur_ori, prev_state, False)
            
        # Enter here exit cleanup code.
        self.brain.saveRwdTable("RwdTable_v7.txt")
        exit(0)

fill_RwdTable()