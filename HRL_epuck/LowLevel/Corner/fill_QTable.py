"""fill_QTable controller."""
# python script to fill QTable for corner in QLearning_square_maze.wbt using best policy from corridor

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent
import json

# print without scientific notation
np.set_printoptions(suppress=True)

class fill_QTable():
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
            
        # Load Positions and respective States of corridor from file
        f1 = open("../Corridor/CorridorPosToStates.txt", "r")
        self.PosStates = json.load(f1)
        self.run()

    def run(self):
        # Values from left to right
        x_vals_r = [x/100 for x in range(-96, -60)]   # corner to right
        x_vals_l = [x/100 for x in range(61, 97)]     # corner to left
        # Values from up to down
        z_vals_1 = [z/100 for z in range(-96, -44)]  # -96 to -74 is Corner, -73 to -45 is corridor
        z_vals_2 = [z/100 for z in range(-96, -73)]
        rot_vals = [x*math.pi/180 for x in range(-180,181)]
        
        # Load corridor QTable
        f = open("../Corridor/QTable_corridor.txt", "r")
        corridorQTable = json.load(f)

        f2 = open("QTABLE_LOG.txt", "w+")
        
        for x in x_vals_r:  # Corner to right  - Uncomment to use
        # for x in x_vals_l:  # Corner to left - Uncomment to use
            if x > -0.74 and x < 0.74:
                z_vals = z_vals_2 # Corner Exit, z can only go up to -74
            else:
                z_vals = z_vals_1 # Corner and corner entrance
            for z in z_vals:
                self.translation_field.setSFVec3f([x,0,z])
                for rot in rot_vals:
                    # Set robot rotation
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

                    # Right corner
                    if x < 0:
                        x_transformed = round(x+0.85,2)
                        z_transformed = round(z+0.85,2) # Corner to right so X tranforms to Z and rotates -90
                    # Left corner
                    else:
                        tmp = round(z+0.85,2)
                        if z < -0.74 and tmp != 0.0:
                            z_transformed = -tmp
                        else:
                            z_transformed = tmp

                        tmp = round(x-0.85,2)
                        if x < 0.74 and tmp != 0.0:
                            x_transformed = -tmp
                        else:
                            x_transformed = tmp
                        
                    # Corner
                    if z < -0.73:    
                        if x < 0:   # Right Corner
                            cur_ori = round((rot*180/math.pi),0)+90  # -90 in corner is equal to 0 in corridor, 0 in corner is 90 in corridor, 180 in corner is -90 in corridor
                        else:       # Left Corner
                            cur_ori = round((rot*180/math.pi),0)-90  

                        if cur_ori > 180:
                            cur_ori -= 360
                        if cur_ori < -180:
                            cur_ori += 360
                        if cur_ori == -180.0:
                            cur_ori = 180.0
                        if cur_ori == -0.0:
                            cur_ori = 0.0

                        # Corner Exit zone
                        if x < 0:   # Right corner   
                            if x > -0.74:
                                if cur_ori > 90:
                                    cur_ori -= 180
                                    if z_transformed != 0.0:
                                        z_transformed = -z_transformed
                                if cur_ori < -90:
                                    cur_ori += 180
                                    if z_transformed != 0.0:
                                        z_transformed = -z_transformed
                                if x > -0.71:
                                    x_transformed = 0.15
                        else:       # Left corner
                            if x < 0.74:
                                if cur_ori > 90:
                                    cur_ori -= 180
                                    if z_transformed != 0.0:
                                        z_transformed = -z_transformed
                                if cur_ori < -90:
                                    cur_ori += 180
                                    if z_transformed != 0.0:
                                        z_transformed = -z_transformed
                                if x < 0.71:
                                    x_transformed = 0.15
                        p = (z_transformed, x_transformed, cur_ori)

                    # Corridor
                    else:
                        if z > -0.74:
                            # -180 < rot < -90 == 0 < rot < 90
                            if rot < -math.pi/2:
                                rot += math.pi
                                if x_transformed != 0.0:
                                    x_transformed = -x_transformed
                            # 90 < rot < 180 == -90 < rot < 0
                            if rot > math.pi/2:
                                rot -= math.pi
                                if x_transformed != 0.0:
                                    x_transformed = -x_transformed
                            if z > -0.71:
                                z_transformed = 0.15
                        
                        cur_ori = round(rot*180/math.pi,0)
                        if cur_ori == -180.0:
                            cur_ori = 180.0
                        if cur_ori == -0.0:
                            cur_ori = 0.0

                        p = (x_transformed, z_transformed, cur_ori)

                    p_x = (x_transformed, z_transformed, cur_ori)                                
                    print('Corner P: ', p, p_x, x, z)
                    
                    corridorState = self.PosStates[str(p)]                  
                    action = self.maxAction(corridorQTable, corridorState)
                    rwd = corridorQTable[corridorState][action]

                    state = self.brain.sensorsToState(dsValues, False)
                    self.brain.QTable[state][action] = rwd
                    
                    for a in self.brain.actions:
                        if a != action and self.brain.QTable[state][a] == 0:
                            self.brain.QTable[state][a] = -500

                    f2.write("POS: " + str(p) + ", " + str((x, z)) + " S: " + state + " CS: " + corridorState + " A: " + action + " R: " + str(rwd) + "\n")

        # Enter here exit cleanup code.
        print("SAVING TABLE...")
        self.brain.saveQTable("QTable_v5_17500h_right.txt")
        print("TABLE SAVED!")
        exit(0)

    def maxAction(self, QTable, state):
        maxAction = max(QTable[state].items(), key=lambda x: x[1])
        listofmaxActions = [k for k, v in QTable[state].items() if v == maxAction[1]]
        return np.random.choice(listofmaxActions)

fill_QTable()