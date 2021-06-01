"""run_Agent controller."""
# python script to control the robot with a QTable after training

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import math
from lowLevel import Agent as lowLvlAgent
from highLevel import Agent as highLvlAgent
import json

# print without scientific notation
np.set_printoptions(suppress=True)

class agentController():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.robot = Supervisor()

        self.robot_node = self.robot.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        # Robot reinforcement learning brain
        # NOTE: epsilon MUST be 0 to disable random actions
        self.frontBrain = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_corridor.txt")  # QTable for corridor and to go forward
        # self.rightBrain = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_right_all.txt") # QTable to go right
        # # Symetry can be used instead (which is better computationally?)
        # self.leftBrain  = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_left_all.txt")  # QTable to go left

        self.leftCorn = lowLvlAgent(epsilon=0, Qtable="../LowLevel/Doors/v52/QTable_corner_v5_17500h_left_sym.txt")
        self.rightCorn = lowLvlAgent(epsilon=0, Qtable="../LowLevel/Doors/v52/QTable_corner_v5_17500h_right.txt")
        self.leftLR = lowLvlAgent(epsilon=0, Qtable="../LowLevel/Doors/v52/QTable_v5_17500h_LR_Left.txt")
        self.rightLR = lowLvlAgent(epsilon=0, Qtable="../LowLevel/Doors/v52/QTable_v5_17500h_LR_Right.txt")
        self.leftFL = lowLvlAgent(epsilon=0, Qtable="../LowLevel/Doors/v52/QTable_v5_17500h_FL_Left.txt")
        self.rightFR = lowLvlAgent(epsilon=0, Qtable="../LowLevel/Doors/v52/QTable_v5_17500h_FR_Right.txt")
        

        self.highLevel = highLvlAgent(alpha=0.3, gamma=0.99, episodes=200, epsilon=0.3, statesFile="maze1.txt")
       
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

        self.leftMotor  = self.robot.getMotor('left wheel motor')
        self.rightMotor = self.robot.getMotor('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.run()

    def run(self):
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        end      = False
        action   = ''
        leftC     = False
        rightC    = False
        rightF = False
        leftF = False
        rightL = False
        leftL = False
        front    = False
        corridor = True

        # HighLvl
        first      = True
        next_State = 0
        cur_State  = 0
        highAction = ''
        last_t = 0
        nr_episodes = 0
        # Robot Start Positions
        # MAZE 1
        # init_pos1 = [0.85,0,0.85]
        # init_ori1 = [0,1,0,1.57]
        # init_pos2 = [-0.85,0,0.65]
        # init_ori2 = [0,1,0,0]
        # init_pos3 = [-0.65,0,-0.85]
        # init_ori3 = [0,1,0,-1.57]
        # init_pos4 = [0.85,0,-0.65]
        # init_ori4 = [0,1,0,3.14]

        # MAZE 2
        init_pos1 = [0.8,0,1.15]
        init_ori1 = [0,1,0,1.57]
        init_pos2 = [-1.15,0,0.6]
        init_ori2 = [0,1,0,0]
        init_pos3 = [-0.7,0,-1.15]
        init_ori3 = [0,1,0,-1.57]
        init_pos4 = [0.65,0,-0.8]
        init_ori4 = [0,1,0,3.14]

        # Set robot intial position
        self.translation_field.setSFVec3f(init_pos1)
        self.rotation_field.setSFRotation(init_ori1)
        self.robot_node.resetPhysics()

        f = open("failed_states_all_tables.txt", "w+")

        while self.robot.step(self.timestep) != -1:     
            # Current position of robot each timestep
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
            cur_ori = round(self.rotation_field.getSFRotation()[3],4)

            # Each timestep check if robot reached goal
            if self.highLevel.reachedGoal((cur_pos[0], cur_pos[2])):
                r_t = round(self.robot.getTime(), 2)
                next_State = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                if not first:
                    self.highLevel.updateQTable(cur_State, next_State, highAction, r_t-last_t)
                else:
                    highAction = self.highLevel.chooseAction(next_State)
                    self.highLevel.updateQTable(next_State, next_State, highAction, r_t-last_t)
                last_t = r_t

                # RESET ROBOT
                cur_State  = 0
                next_State = 0
                first = True
                nr_episodes += 1
                if nr_episodes < 25:
                    self.translation_field.setSFVec3f(init_pos1)
                    self.rotation_field.setSFRotation(init_ori1)
                elif nr_episodes < 50:
                    self.translation_field.setSFVec3f(init_pos2)
                    self.rotation_field.setSFRotation(init_ori2)
                elif nr_episodes < 75:
                    self.translation_field.setSFVec3f(init_pos3)
                    self.rotation_field.setSFRotation(init_ori3)
                else:
                    self.translation_field.setSFVec3f(init_pos4)
                    self.rotation_field.setSFRotation(init_ori4)
                self.robot_node.resetPhysics()
                print("EPISODE ", nr_episodes)

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

            # Get distances of sensors' voltages
            F  = self.frontBrain.sensorVoltageToDistance(dsValues[0])  # Front
            FL = self.frontBrain.sensorVoltageToDistance(dsValues[1])  # Front Left
            L  = self.frontBrain.sensorVoltageToDistance(dsValues[2])  # Left
            FR = self.frontBrain.sensorVoltageToDistance(dsValues[3])  # Front Right
            R  = self.frontBrain.sensorVoltageToDistance(dsValues[4])  # Right
            B  = self.frontBrain.sensorVoltageToDistance(dsValues[5])  # Back

            # DETECT DOOR OR CORNER
            # Left Right Door
            if FR == 0.3 and FL == 0.3 and corridor:
                # highAction = np.random.choice(["left", "right"])
                
                # High Level Training
                r_t = round(self.robot.getTime(), 2)
                # State reached after last highAction (each position has a margin of 3cm since the robot won't always do the same path)
                next_State = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                print("LR", next_State)
                
                if next_State == '':
                    f.write(str((cur_pos[0], cur_pos[2], cur_ori)) + " in LR DOOR")
                    highAction = np.random.choice(["left", "right"])
                else: 
                    # Updates occur only after executing the first highLevel action (so that cur_state and next_state exist)
                    if not first:
                        self.highLevel.updateQTable(cur_State, next_State, highAction, r_t-last_t)
                    
                    # New QLearning cicle starts here
                    cur_State = next_State
                    highAction = self.highLevel.chooseAction(cur_State)
                    first = False        
                    last_t = r_t
                
                if highAction == "right":
                    print("ROBOT REACHED LR DOOR AND GOES RIGHT " + str((cur_pos[0], cur_pos[2])))
                    rightL = True
                    leftL  = False
                elif highAction == "left":
                    print("ROBOT REACHED LR DOOR AND GOES LEFT " + str((cur_pos[0], cur_pos[2])))
                    rightL = False
                    leftL  = True
                front     = False
                corridor  = False
                rightC = False
                rightF = False
                leftC = False
                leftF = False
                
            # Front Right door/Corridor
            elif FR == 0.3 and R == 0.3 and B == 0.3:
                if corridor:
                    # Front Right Door
                    if F == 0.3:
                        # highAction = np.random.choice(["front", "right"])
                        
                        # High Level Training
                        r_t = round(self.robot.getTime(), 2)
                        # State reached after last highAction 
                        next_State = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                        print("FR", next_State)
        
                        if next_State == '':
                            f.write(str((cur_pos[0], cur_pos[2], cur_ori)) + " in FR DOOR")
                            highAction = np.random.choice(["front", "right"])
                        else:
                            # Updates occur only after executing the first highLevel action (so that cur_state and next_state exist)
                            if not first:
                                self.highLevel.updateQTable(cur_State, next_State, highAction, r_t-last_t)

                            # New QLearning cicle starts here
                            cur_State = next_State
                            highAction = self.highLevel.chooseAction(cur_State)
                            first = False        
                            last_t = r_t

                        if highAction == "right":
                            print("ROBOT REACHED FR DOOR AND GOES RIGHT " + str((cur_pos[0], cur_pos[2])))
                            rightF = True
                            front = False
                        elif highAction == "front":
                            print("ROBOT REACHED FR DOOR AND GOES FORWARD " + str((cur_pos[0], cur_pos[2])))
                            front = True
                            rightF = False
                        rightC = False
                    # Corner to Right
                    else:
                        # print("ROBOT REACHED RIGHT CORNER")
                        rightC = True
                        front = False
                        rightF = False
                    corridor  = False
                    leftC = False
                    leftF = False
                    rightL = False
                    leftL = False

            # Front Left door/Corridor
            elif FL == 0.3 and L == 0.3 and B == 0.3:
                if corridor:
                    # Front Left Door
                    if F == 0.3:  
                        # highAction = np.random.choice(["front", "left"])
                        
                        # High Level Training
                        r_t = round(self.robot.getTime(), 2)
                        # State reached after last highAction 
                        next_State = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                        print("FL", next_State)
        
                        if next_State == '':
                            f.write(str((cur_pos[0], cur_pos[2], cur_ori)) + " in FL DOOR")
                            highAction = np.random.choice(["front", "left"])
                        else:
                            # Updates occur only after executing the first highLevel action (so that cur_state and next_state exist)
                            if not first:
                                self.highLevel.updateQTable(cur_State, next_State, highAction, r_t-last_t)
                            
                            # New QLearning cicle starts here
                            cur_State = next_State
                            highAction = self.highLevel.chooseAction(cur_State)
                            first = False        
                            last_t = r_t

                        if highAction == "left":
                            print("ROBOT REACHED FL DOOR AND GOES LEFT " + str((cur_pos[0], cur_pos[2])))
                            leftF  = True
                            front = False
                        elif highAction == "front":
                            print("ROBOT REACHED FL DOOR AND GOES FORWARD " + str((cur_pos[0], cur_pos[2])))
                            front = True
                            leftF  = False
                        leftC = False
                    # Left Corner
                    else:
                        # print("ROBOT REACHED LEFT CORNER")
                        leftC  = True
                        front = False
                        leftF = False
                    corridor  = False
                    rightC = False
                    rightF = False
                    rightL = False
                    leftL = False    
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R <= 0.2 and L <= 0.2 and R < FR and L < FL and B == 0.3 and F == 0.3 and (leftC or rightC or leftF or rightF or leftL or rightL or front):
                # print("ROBOT EXITS DOOR/CORNER AND IS ON CORRIDOR")
                front    = False
                leftC     = False
                rightC    = False
                leftF    = False
                rightF    = False
                leftL     = False
                rightL    = False
                corridor = True

            # Current state of the robot
            state = self.frontBrain.sensorsToState(dsValues)        
            if leftC and not rightC and not leftL and not rightL and not leftF and not rightF and not front and not corridor:
                # print("Turn Left")
                action = self.leftCorn.chooseAction(state) # best action in current state
            elif not leftC and rightC and not leftL and not rightL and not leftF and not rightF and not front and not corridor:
              # print("Turn Right")
                action = self.rightCorn.chooseAction(state)
            elif leftL and not rightC and not leftC and not rightL and not leftF and not rightF and not front and not corridor:
                # print("Turn Left")
                action = self.leftLR.chooseAction(state) # best action in current state
            elif not leftC and rightL and not leftL and not rightC and not leftF and not rightF and not front and not corridor:
              # print("Turn Right")
                action = self.rightLR.chooseAction(state)
            elif leftF and not rightC and not leftL and not rightL and not leftC and not rightF and not front and not corridor:
                # print("Turn Left")
                action = self.leftFL.chooseAction(state) # best action in current state
            elif not leftC and rightF and not leftL and not rightL and not leftF and not rightC and not front and not corridor:
              # print("Turn Right")
                action = self.rightFR.chooseAction(state)
            elif (front or corridor) and not rightC and not leftC and not leftL and not rightL and not leftF and not rightF:
                # if front:
                    # print("Go Forward")
                action = self.frontBrain.chooseAction(state)
            else:
                print("ONLY ONE SHOULD BE TRUE F:", front, " rightC:", rightC, " leftC:", leftC, " rightL:", rightL, " leftL:", leftL, " rightF:", rightF, " leftF:", leftF, " C:", corridor)
                end = True
                break
            speeds = self.frontBrain.actionToSpeed(action)   # speed of each motor
            # print("State: ", state, "Action: ", action)

            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.1:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break    
            if end or nr_episodes > self.highLevel.episodes:
                break

        print("SAVING TABLE...")
        self.highLevel.saveQTable("QTable_HighLevel_all_tables.txt")
        print("TABLE SAVED!")
        # Enter here exit cleanup code.
        exit(0)

agentController()