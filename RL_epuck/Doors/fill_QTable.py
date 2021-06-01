"""fill_QTable controller."""
# python script to fill QTable for Doors in QLearning_doors.wbt using best policy from corridor

from controller import DistanceSensor, Supervisor
import numpy as np
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
        f1 = open("CorridorPosToStates.txt", "r")
        self.PosStates = json.load(f1)

        # Create all doors and fill a QTable for each action sequentially
        self.run()    

    def run(self):
        # Fill QTables for all doors 
        doorTypes = ["FR", "LR"]  # FLR dropped, FL obtained with symmetry from FR
        x_vals_r = [x/100 for x in range(-11, 21)]   # Corridor side to side and door exit
        x_vals_fwd = [x/100 for x in range(-11, 12)] # Corridor side to side
        x_vals_lr = [x/100 for x in range(-11, 12)]  # Corridor middle to right side (LR Door is symmetrical in X = 0)
    
        z_vals_r = [z/100 for z in range(0, 12)]    # Corridor center of FR Door to bottom (FR Door is symmetrical in Z = 0)
        z_vals_lr = [z/100 for z in range(-11, 25)]  # Inside LR Door
        z_vals_fwd_lr = [z/100 for z in range(11, 35)]
        z_vals_fwd = [z/100 for z in range(-25, 26)] # Door entrance up to door exit
        z_vals_fwd += [z/100 for z in range(74, 95)] # Dead End
        
        rot_vals_r = [rot*math.pi/180 for rot in range(-180, 0)]    # Used in FR Door -> 90 to 180 degrees ignored in FR Door action to right (assumed positions won't happen)
        rot_vals_fwd = [rot*math.pi/180 for rot in range(-180, 180)] # 0 to 360 degrees used in FR Door action to front (angle conversion makes this table suitable for FL Door aswell (270 to 90 is FR Door, 90 to 270 is FL Door))
        rot_vals_lr = [rot*math.pi/180 for rot in range(-180, 0)]   # Used in LR Door
        rot_vals_fwd_lr = [rot*math.pi/180 for rot in range(-90, 90)] # 0 to 360 degrees used in FR Door action to front (angle conversion makes this table suitable for FL Door aswell (270 to 90 is FR Door, 90 to 270 is FL Door))

        # Load corridor QTable
        f = open("QTable_v5_22000h.txt", "r")
        corridorQTable = json.load(f)

        doorTypes = ["FR"]
        for door in doorTypes:
            if door == "FR":
                # Set wall on the left side
                self.wall_pos.setSFVec3f([-0.155,0,0])
                self.wall_rot.setSFRotation([0,1,0,-1.5708])
                # actions = ["Front", "Right"]
                actions = ["Right"]
            elif door == "LR":
                # Set wall in front of corridor
                self.wall_pos.setSFVec3f([0,0,-0.155])
                self.wall_rot.setSFRotation([0,1,0,0])
                actions = ["Right"] # Left turn obtained using symmetry     
                # actions = ["Front"]      
            for act in actions:
                f2 = open("QTABLE_LOG_"+door+"_"+act+".txt", "w+")
                # Robot reinforcement learning brain: Create new Qtable for each door and action
                self.brain = Agent(sensors_states=6)
                
                if act == "Front":
                    x_vals = x_vals_fwd
                    z_vals = z_vals_fwd
                    rot_vals = rot_vals_fwd
                    if door == "LR":
                        z_vals = z_vals_fwd_lr
                        rot_vals = rot_vals_fwd_lr
                elif act == "Right":
                    if door == "LR":
                        x_vals = x_vals_lr
                        z_vals = z_vals_lr
                        rot_vals = rot_vals_lr
                    else:
                        x_vals = x_vals_r
                        z_vals = z_vals_r    
                        rot_vals = rot_vals_r
                for x in x_vals:
                    for z in z_vals:
                        if z > 0.15 and door == "LR": 
                            rot_vals = rot_vals_fwd_lr
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
                                
                                # Corridor dead-end
                                if z >= 0.74:
                                    z_transformed = round(0.85-z,2)
                                    p = (x_transformed, z_transformed, cur_ori)
                                else:
                                    # FR and FL Doors entrance
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
                                    p = (x_transformed, 0.0, cur_ori)  # 0.15 is a position considered to be middle of corridor (situation where rotation is symmetric: -80º=100º)
                            else:
                                cur_ori = round((rot*180/math.pi),0)+90
                                z_transformed = z
                                x_transformed = x
                                if x_transformed > 0.11:
                                    x_transformed = 0.11

                                if cur_ori > 180:
                                    cur_ori -= 360
                                if cur_ori < -180:
                                    cur_ori += 360
                                if cur_ori == -180.0:
                                    cur_ori = 180.0
                                if cur_ori == -0.0:
                                    cur_ori = 0.0

                                if z > 0.11:
                                    cur_ori = round((rot*180/math.pi),0)+30
                                    if cur_ori > 180:
                                        cur_ori -= 360
                                    if cur_ori < -180:
                                        cur_ori += 360
                                    if cur_ori == -180.0:
                                        cur_ori = 180.0
                                    if cur_ori == -0.0:
                                        cur_ori = 0.0
                                    if cur_ori < 90 and cur_ori > -90:        
                                        p = (x, 0.0, cur_ori)
                                    else:
                                        p = (x, -0.07, cur_ori)
                                else:
                                    if cur_ori < 90 and cur_ori > -90:        
                                        p = (z_transformed, 0.0, cur_ori)
                                    else:
                                        p = (z_transformed, -0.07, cur_ori)

                            corridorState = self.PosStates[str(p)]                  
                            action = self.maxAction(corridorQTable, corridorState)
                            rwd = corridorQTable[corridorState][action]
                            state = self.brain.sensorsToState(dsValues, False)
                            
                            # if state not in table:
                            #     table[state] = action
                            
                            self.brain.QTable[state][action] = rwd
                            
                            # Put non achieved states at -500 to isolate using maxAction method
                            for a in self.brain.actions:
                                if a != action and self.brain.QTable[state][a] == 0:
                                    self.brain.QTable[state][a] = -500

                            print(act, "POS: ", p, state, action)
                            f2.write("POS: " + str(p) + ", POS_B4: " + str((x, z, rot)) + "\nS: " + state + " CS: " + corridorState + " A: " + action + " RWD: " + str(rwd) + "\n")
                f2.close()          
                print("SAVING TABLE "+door+" "+act+"... ",)
                self.brain.saveQTable("QTable_22000h_"+door+"_"+act+".txt")
                # json.dump(table, open("QTable_22000h_"+door+"_"+act+".txt", "w+"))
                print("TABLE SAVED!")

        # Enter here exit cleanup code.
        exit(0)

    def maxAction(self, QTable, state):
        maxAction = max(QTable[state].items(), key=lambda x: x[1])
        listofmaxActions = [k for k, v in QTable[state].items() if v == maxAction[1]]
        return np.random.choice(listofmaxActions)

fill_QTable()