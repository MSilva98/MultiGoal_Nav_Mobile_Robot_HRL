"""corridorPosToStates controller."""
# python script to get the states associated to each position in QLearning_giant_corridr.wbt

from os import stat
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent
import json

# print without scientific notation
np.set_printoptions(suppress=True)

class corridorPosToStates():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()

        # node to use supervisor functions
        self.robot_node = self.supervisor.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        self.initial_pos = self.translation_field.getSFVec3f()
        self.initial_rot = self.rotation_field.getSFRotation()

        self.brain = Agent()
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

        # Get positions in corridor
        self.getCorridorPosStates()

    def getCorridorPosStates(self):
        x_vals = [x/100 for x in range(-11, 12)]
        z_vals = [z/100 for z in range(120, 147)]
        rot_vals = [x*math.pi/180 for x in range(-180,181)]   # Corridor states are diagonally symmetrical
        self.PosStates = dict()
        
        for z in z_vals:
            for x in x_vals:
                self.translation_field.setSFVec3f([x,0,z])
                for rot in rot_vals:
                    self.rotation_field.setSFRotation([0,1,0,rot])
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

                    x_transformed = x
                    z_transformed = round(1.35-z,2)  # z = 1.24 = 11, z = 1.46 = -11
                    cur_ori = round(rot*180/math.pi,0)

                    if cur_ori > 180:
                        cur_ori -= 360
                    if cur_ori < -180:
                        cur_ori += 360
                    if cur_ori == -180.0:
                        cur_ori = 180.0
                    if cur_ori == -0.0:
                        cur_ori = 0.0

                    if z_transformed <= 0.12:  
                        p = (x_transformed, z_transformed, cur_ori)
                    else:
                        # Dead-End
                        # -180 < rot < -90 == 0 < rot < 90
                        if cur_ori < -90:
                            cur_ori += 180
                            if x_transformed != 0.0:
                                x_transformed = -x_transformed
                        # 90 < rot < 180 == -90 < rot < 0
                        if cur_ori > 90:
                            cur_ori -= 180
                            if x_transformed != 0.0:
                                x_transformed = -x_transformed
                        p = (x_transformed, z_transformed, cur_ori)

                    state = self.brain.sensorsToState(dsValues, False)
                    print('Corridor P: ', p, state)

                    if p not in self.PosStates:
                        self.PosStates[str(p)] = state

        with open("CorridorPosToStates.txt", "w+") as jsonFile:
            json.dump(self.PosStates, jsonFile)

corridorPosToStates()