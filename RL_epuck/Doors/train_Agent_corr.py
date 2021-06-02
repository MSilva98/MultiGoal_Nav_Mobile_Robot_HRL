"""train_Agent controller."""
# python script to train the robot with a given RWD Table

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

        # Robot reinforcement learning brain
        self.brain = Agent(alpha=0.3, beta=0.05, ro=0, epsilon=0.4, sensors_states=6, QTable="QTable_v6_6000h.txt", RWDTable="RwdTable_v6.txt")

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
        last_t = 0
        newRandom = False

        x_space = [x/100 for x in range(-11, 12, 3) if abs(x) > 1]
        z_space = [z/100 for z in range(125, 147, 5)] # giant corridor 3m
        # z_space = [z/100 for z in range(30, 56, 5)]   # corridor 1.25m
        ori_space = [o*math.pi/180 for o in range(0, 360, 15)]
        start_pos = []

        while self.supervisor.step(self.timestep) != -1:
            cur_pos = abs(round(self.translation_field.getSFVec3f()[0],2))
            cur_ori = abs(round(self.rotation_field.getSFRotation()[3]*180/math.pi,0))
            if cur_ori >= 180:
                cur_ori -= 180

            newRandom = False
            t = self.supervisor.getTime()
            r_t = round(t,0)
            # New random position when centered and 5 min passed
            if r_t-last_t >= 300:
                last_t = r_t
                print("NEW RANDOM POSITION ", last_t)
                x = np.random.choice(x_space)
                z = np.random.choice(z_space)
                ori = np.random.choice(ori_space)
                p = (x,z,ori*180/math.pi)
                start_pos.append(p)
                self.translation_field.setSFVec3f([x,0,z])
                self.rotation_field.setSFRotation([0,1,0,ori])
                self.robot_node.resetPhysics()
                newRandom = True
        
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
            
            if not first and not newRandom:   # first step won't update table before take action
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

                first = False
                
            if round(t,0) == 7200000:
                print("SAVING TABLE...")
                self.brain.saveQTable("QTable_v6_8000h.txt")
                print("TABLE SAVED!")
                        
            if round(t,0) == 14400000:
                print("SAVING TABLE...")
                self.brain.saveQTable("QTable_v6_10000h.txt")
                print("TABLE SAVED!")
            
            # Controller exit or t = 6000h 
            if end or round(t,0) == 21600000:
                break
        
        print("Positions:", len(start_pos))
        # Enter here exit cleanup code.
        print("SAVING TABLE...")
        self.brain.saveQTable("QTable_v6_12000h.txt")
        print("TABLE SAVED!")
        exit(0)

agentController()