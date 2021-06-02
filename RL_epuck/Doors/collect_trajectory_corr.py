"""corridor_analysis controller."""
# python script to obtain the robot error to the optimal position

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from agent import Agent
import json

# print without scientific notation
np.set_printoptions(suppress=True)

class agentController():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.robot = Supervisor()
        # node to use supervisor functions
        self.robot_node = self.robot.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        self.initial_pos = self.translation_field.getSFVec3f()
        self.initial_rot = self.rotation_field.getSFRotation()

        # Robot reinforcement learning brain
        self.brain = Agent(alpha=0, beta=0, ro=0, epsilon=0, Qtable="QTable_v6_6000h.txt")

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
            self.ds.append(self.robot.getDistanceSensor(self.dsNames[i]))
            self.ds[i].enable(self.timestep)

        self.leftMotor = self.robot.getMotor('left wheel motor')
        self.rightMotor = self.robot.getMotor('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.run()

    def run(self):
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        end = False
        action = ''
        all_pos = []
        all_starts = []
        last_t = 0

        x_space = [x/100 for x in range(-9, 10, 3) if x != 0]  # -9, -6, -3, 3, 6, 9
        ori_space = [o*math.pi/180 for o in range(-60, 61, 15) if o != 0] # -60, -45, -30, -15, 15, 30, 45, 60
        iterations = len(ori_space)*len(x_space)
        visited_pos = []

        # Start with convergence mode
        convergence = True
        init_t = 0
        while self.robot.step(self.timestep) != -1:
            cur_pos = [round(p,4) for p in self.translation_field.getSFVec3f()]

            # When all convergence states done go to X error mode
            if len(visited_pos) >= iterations and abs(round(cur_pos[0],3)) <= 0.007 and convergence:
                convergence = False
                init_t = self.robot.getTime()
                print("X Error mode")

            if abs(round(cur_pos[0],3)) <= 0.01 and convergence:
                init_t = self.robot.getTime()
                x = np.random.choice(x_space)
                ori = np.random.choice(ori_space)
                if (x, ori) not in visited_pos:
                    all_starts.append((-5,-5))
                    visited_pos.append((x,ori))
                    self.translation_field.setSFVec3f([x,0,1])
                    self.rotation_field.setSFRotation([0,1,0,ori])
                    self.robot_node.resetPhysics()
                    print("NEW POSITION")
                
            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.05:
                # read sharp sensors outputs
                dsValues = []
                for i in range(len(self.ds)):
                    dsValues.append(self.ds[i].getValue())
                
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break
            
            cur_pos = [round(p,4) for p in self.translation_field.getSFVec3f()]

            state = self.brain.sensorsToState(dsValues, False) # next state - state after action                    
            action = self.brain.chooseAction(state)     # best action in current state
            speeds = self.brain.actionToSpeed(action)   # speed of each motor
            print("State: ", state, "Action: ", action)
            
            if round(t,0) - last_t == 1:
                last_t = round(t,0)
                if not convergence:
                    all_pos.append((cur_pos[0], cur_pos[2]))
                else:
                    all_starts.append((cur_pos[0], cur_pos[2]))

            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.1:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break

            # End or 2h passed after all convergence states visited
            if end or round(t,0)-init_t >= 3600:
                break
        
        print("SAVING TABLES...")
        with open("agent_positions.txt", "w+") as f:
            all_pos.append((-5,-5))
            for p in all_pos:
                f.write(str(p) + "\n")
        print("AGENT_POSITIONS SAVED!")

        with open("agent_convergence.txt", "w+") as f:
            for p in all_starts:
                f.write(str(p) + "\n")
        print("AGENT_CONVERGENCE SAVED!")

        # Enter here exit cleanup code.
        exit(0)

agentController()