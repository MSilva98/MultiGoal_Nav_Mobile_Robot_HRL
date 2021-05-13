"""fill_QTable controller."""
# python script to fill QTable for Doors in QLearning_square_maze.wbt using best policy from corridor

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
        
        # node to use supervisor functions
        self.robot_node = self.supervisor.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        self.initial_pos = self.translation_field.getSFVec3f()
        self.initial_rot = self.rotation_field.getSFRotation()

        self.wall_node = self.supervisor.getFromDef("wallDoors")
        self.wall_pos = self.wall_node.getField("translation")
        self.wall_rot = self.wall_node.getField("rotation")

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
        # File generated using getCorridorPosStates.py
        f1 = open("../Corridor/CorridorPosToStates.txt", "r")
        self.PosStates = json.load(f1)

        # Create all doors and fill a QTable for each action sequentially
        self.run()    
        # self.runOthers()

    def run(self):
        # Fill QTables for all doors 
        doorTypes = ["FR", "FL", "LR"]  # FLR dropped
        x_vals = [x/100 for x in range(-11, 20)] # Corridor side to side
        z_vals_l_r = [z/100 for z in range(0, 12)] # Corridor door up to bottom
        z_vals_fwd = [z/100 for z in range(-20, 32)] # Door entrance up to door exit
        z_vals_fwd += [z/100 for z in range(74, 94)] # Dead End
        rot_vals_l_r = [rot*math.pi/180 for rot in range(90, -180, -1)]
        rot_vals_fwd = [rot*math.pi/180 for rot in range(0, 360)]
        
        # Load corridor QTable
        # f = open("../Corridor/QTable_corrected_v5_6s_4e_5000h_once.txt", "r")
        f = open("../Corridor/QTable_v5_13000h_corrected.txt", "r")
        f = open("../Corridor/QTable_v5_17500h_corrected.txt", "r")
        corridorQTable = json.load(f)

        doorTypes = ["LR"]
        for door in doorTypes:
            if door == "FR":
                # Set wall on the left side
                self.wall_pos.setSFVec3f([-0.155,0,0])
                self.wall_rot.setSFRotation([0,1,0,-1.5708])
                # actions = ["Front", "Right"]
                actions = ["Right"]
            elif door == "FL":
                # Set wall on the right side
                self.wall_pos.setSFVec3f([0.155,0,0])
                self.wall_rot.setSFRotation([0,1,0,-1.5708])
                # actions = ["Front", "Left"]
                actions = ["Left"]
            elif door == "LR":
                # Set wall in front of corridor
                self.wall_pos.setSFVec3f([0,0,-0.155])
                self.wall_rot.setSFRotation([0,1,0,0])
                # actions = ["Left", "Right"]
                # actions = ["Right", "Front"]
                actions = ["Front"]
                # actions = ["Right"]
            
            elif door == "FLR":
                # Set wall away of corridor
                self.wall_pos.setSFVec3f([0.3,0,0.4])
                self.wall_rot.setSFRotation([0,1,0,-1.5708])
                actions = ["Front", "Left", "Right"]
                
            for act in actions:
                f2 = open("./v5/QTABLE_LOG_"+door+"_"+act+".txt", "w+")
                # Robot reinforcement learning brain
                self.brain = Agent(sensors_states=6)
                
                if act == "Front":
                    x_vals = [x/100 for x in range(-11, 12)]
                    if door == "LR":
                        z_vals = [z/100 for z in range(15, 35)]
                        # rot_vals = [rot*math.pi/180 for rot in range(-90, 90)]
                        rot_vals = [0]
                    else:
                        z_vals = z_vals_fwd
                        rot_vals = rot_vals_fwd
                elif act == "Right" or act == "Left":
                    rot_vals = rot_vals_l_r
                    if door == "LR":
                        x_vals = [x/100 for x in range(0, 12)]
                        z_vals = [z/100 for z in range(-11, 20)]
                    else:
                        z_vals = z_vals_l_r

                for x in x_vals:
                    for z in z_vals:
                        # if x > 0.11 or x < -0.11:
                        #     if z > 0.11:
                        #         z = 0.11
                        #     elif z < -0.11:
                        #         z = -0.11
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
                            
                            if act == "Front":
                                x_transformed = x
                                cur_ori = round(rot*180/math.pi,0)
                                
                                if cur_ori > 180:
                                    cur_ori -= 360
                                if cur_ori < -180:
                                    cur_ori += 360
                                if cur_ori == -180.0:
                                    cur_ori = 180.0
                                if cur_ori == -0.0:
                                    cur_ori = 0.0
                                
                                if door == "LR":
                                    p = (x_transformed, 0.15, cur_ori)
                                else:
                                    # Corridor dead-end
                                    if z >= 0.73:
                                        z_transformed = round(0.85-z,2)
                                        p = (x_transformed, z_transformed, cur_ori)
                                    else:
                                        # Door entrance
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
                                        p = (x_transformed, 0.15, cur_ori)
                            else:
                                # Turn Right on door (LR or FR)
                                # if act == "Right":
                                #     cur_ori = round((rot*180/math.pi),0)+90
                                #     z_transformed = z
                                # # Turn Left on door (LR or FL)
                                # elif act == "Left":
                                #     cur_ori = round((rot*180/math.pi),0)-90
                                #     if z != 0.0:
                                #         z_transformed = -z
                                if z <= 0.11:
                                    cur_ori = round((rot*180/math.pi),0)+90
                                    z_transformed = z

                                    if cur_ori > 180:
                                        cur_ori -= 360
                                    if cur_ori < -180:
                                        cur_ori += 360
                                    if cur_ori == -180.0:
                                        cur_ori = 180.0
                                    if cur_ori == -0.0:
                                        cur_ori = 0.0

                                    if cur_ori < 90 and cur_ori > -90:
                                        p = (z_transformed, 0.11, cur_ori)
                                    else:
                                        p = (z_transformed, -0.05, cur_ori)
                                else:
                                    cur_ori = round((rot*180/math.pi),0)
                                    p = (x, 0.11, cur_ori)

                            # print(act,"POS: ", p, x, z, round(rot*180/math.pi,0))

                            corridorState = self.PosStates[str(p)]                  
                            action = self.maxAction(corridorQTable, corridorState)
                            rwd = corridorQTable[corridorState][action]

                            print(act, "POS: ", p, x, z, round(rot*180/math.pi,0), action)

                            state = self.brain.sensorsToState(dsValues, False)
                            self.brain.QTable[state][action] = rwd
                            
                            for a in self.brain.actions:
                                if a != action and self.brain.QTable[state][a] == 0:
                                    self.brain.QTable[state][a] = -500
        
                            f2.write("POS: " + str(p) + ", POS_B4: " + str((x, z, rot)) + "\nS: " + state + " CS: " + corridorState + " A: " + action + " RWD: " + str(rwd) + "\n")
                f2.close()          
                print("SAVING TABLE "+door+" "+act+"... ",)
                self.brain.saveQTable("./v5/QTable_v5_17500h_corrected_"+door+"_"+act+".txt")
                # self.brain.saveQTable("./v4/QTable_v5_13000h_"+act+".txt")
                print("TABLE SAVED!")

        # Enter here exit cleanup code.
        exit(0)

    def maxAction(self, QTable, state):
        maxAction = max(QTable[state].items(), key=lambda x: x[1])
        listofmaxActions = [k for k, v in QTable[state].items() if v == maxAction[1]]
        return np.random.choice(listofmaxActions)

fill_QTable()