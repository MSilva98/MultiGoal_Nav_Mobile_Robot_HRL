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

class fill_QTable():
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
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        rot = 0
        while self.supervisor.step(self.timestep) != -1:
            # read sharp sensors outputs
            dsValues = []
            for i in range(len(self.ds)):
                dsValues.append(self.ds[i].getValue())

            print("Value: ", self.brain.getPosOriFromSensors(dsValues))

            # add initial reward to that state in QTable
            # dsValues = [front, front left, left, front right, right, rear]
            # self.brain.fillRwdTable(dsValues)

            # # rotate 1 degree until 360º
            # rot += math.pi/180
            # self.initial_rot[3] = rot
            
            # if rot >= math.pi*2:    # after rotating 360º translate 5mm to the right
            #     rot = 0
            #     self.initial_pos[0] = self.initial_pos[0]+0.005
            #     if self.initial_pos[0] > 0.11:
            #         break
            #     self.translation_field.setSFVec3f(self.initial_pos)
            # else:
            #     self.rotation_field.setSFRotation(self.initial_rot)
            
            # self.initial_pos[0] = self.initial_pos[0]+0.06
            # self.initial_rot[3] = math.pi/2
            # self.translation_field.setSFVec3f(self.initial_pos)
            # self.rotation_field.setSFRotation(self.initial_rot)

            break



        # Enter here exit cleanup code.
        # self.brain.saveQTable()
        exit(0)

fill_QTable()