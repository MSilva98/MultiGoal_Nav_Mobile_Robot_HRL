"""fill_QTable controller."""
# python script to collect data from Sharp and infrared distance sensors

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
        prev_state = "(1, 1, 1, 1, 1, 1)"
        # Values from left wall to right wall of corridor
        # x_vals = [x/1000 for x in range(-110, 111)] # 1mm
        x_vals = [x/100 for x in range(-11, 12)]  # 1cm
        # Values 4cm up to 33cm of front wall 1cm apart
        z_vals = [z/100 for z in range(-146, -116)] # 4 to 33cm
        # Middle of corridor
        z_vals.append(0) 
        # 360 degrees in radians
        rot_vals = [x*math.pi/180 for x in range(0,361)] # rotate only 180 degrees
        
        # for lst in zlist:
        for z in z_vals:
            for x in x_vals:
                # Set robot position
                self.translation_field.setSFVec3f([x,0,z])
                for rot in rot_vals:
                    # Set robot rotation
                    self.rotation_field.setSFRotation([0,1,0,rot])

                    t = self.supervisor.getTime()
                    # 0.05 guarantees a read of actual values of the sensors
                    while self.supervisor.getTime() - t < 0.05:
                        # print("BF:",self.supervisor.getTime())
                        # read sharp sensors outputs
                        dsValues = []
                        for i in range(len(self.ds)):
                            dsValues.append(self.ds[i].getValue())
                        # print("AF:",self.supervisor.getTime())

                        # controller termination
                        if self.supervisor.step(self.timestep) == -1:
                            quit()
                        
                    cur_pos = [round(p,4) for p in self.translation_field.getSFVec3f()]
                    cur_ori = round(self.rotation_field.getSFRotation()[3],4)

                    # Trimm rotation between -90 and 90
                    # 100 degrees at x = -0.028 is the same as -80 at x = 0.028
                    # Important when calculating RWD
                    if cur_ori*180/math.pi < -90:
                        cur_ori += math.pi
                        cur_pos[0] = -cur_pos[0]
                    if cur_ori*180/math.pi > 90:
                        cur_ori -= math.pi      
                        cur_pos[0] = -cur_pos[0]               
                    
                    # add initial reward to that state in QTable
                    # dsValues = [front, front left, left, front right, right, rear]
                    prev_state = self.brain.fillRwdTable(dsValues, cur_pos, cur_ori, prev_state)
        
        # Enter here exit cleanup code.
        self.brain.saveRwdTable("RwdTable_v2_1cm.txt")
        exit(0)

fill_RwdTable()