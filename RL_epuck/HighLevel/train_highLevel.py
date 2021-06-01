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
        self.rightBrain = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_right_all.txt") # QTable to go right
        # Symetry can be used instead (which is better computationally?)
        self.leftBrain  = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_left_all.txt")  # QTable to go left
       
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
        left     = False
        right    = False
        front    = False
        corridor = True

        #TMP
        corners = False

        # HighLvl
        first      = True
        next_State = 0
        cur_State  = 0
        highAction = ''
        last_t = 0
        nr_episodes = 0
        
        # Select maze
        maze = 1

        # Robot Start Positions
        if maze == 1:
            # MAZE 1
            self.highLevel = highLvlAgent(alpha=0.3, gamma=0.99, episodes=300, epsilon=0.3, statesFile="./mazes/maze1.txt")
            init_pos1 = [0.85,0,0.85]
            init_ori1 = [0,1,0,1.57]
            init_pos2 = [-0.85,0,0.65]
            init_ori2 = [0,1,0,0]
            init_pos3 = [-0.65,0,-0.85]
            init_ori3 = [0,1,0,-1.57]
            init_pos4 = [-0.15,0,-0.55]
            init_ori4 = [0,1,0,-1.57]
            Qtable_name = "QTable_HighLevel_maze1.txt"
            avg_lowLvl_name = "avgLowLvlTimes_maze1.txt"
            visitedStates_name = "visited_states_maze1.txt"
            paths_name = "steps_maze1.txt"
            rewardSum_name = "rwdSum_maze1.txt"
        else:
            # MAZE 2
            self.highLevel = highLvlAgent(alpha=0.3, gamma=0.99, episodes=500, epsilon=0.3, statesFile="./mazes/maze2.txt")
            init_pos1 = [0.8,0,1.15]
            init_ori1 = [0,1,0,1.57]
            init_pos2 = [-1.15,0,0.6]
            init_ori2 = [0,1,0,0]
            init_pos3 = [-0.7,0,-1.15]
            init_ori3 = [0,1,0,-1.57]
            init_pos4 = [1.15,0,-0.8]
            init_ori4 = [0,1,0,3.14]
            Qtable_name = "QTable_HighLevel_maze2.txt"
            avg_lowLvl_name = "avgLowLvlTimes_maze2.txt"
            visitedStates_name = "visited_states_maze2.txt"
            paths_name = "steps_maze2.txt"
            rewardSum_name = "rwdSum_maze2.txt"

        # Set robot intial position
        self.translation_field.setSFVec3f(init_pos1)
        self.rotation_field.setSFRotation(init_ori1)
        self.robot_node.resetPhysics()

        # Metrics
        visited_states = dict()
        paths = dict()
        statesPath = []
        avg_lowLvlTimes = []
        rewardSum = [0]*self.highLevel.episodes

        while self.robot.step(self.timestep) != -1:     
            # Current position of robot each timestep
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
            cur_ori = round(self.rotation_field.getSFRotation()[3],2)

            # Each timestep check if robot reached goal
            if self.highLevel.reachedGoal((cur_pos[0], cur_pos[2], cur_ori)):
                r_t = round(self.robot.getTime(), 2)
                next_State = self.highLevel.getState((cur_pos[0], cur_pos[2], cur_ori), checkingGoal=True)
                if next_State != '':
                    if next_State not in visited_states:
                        visited_states[next_State] = 1
                    else:
                        visited_states[next_State] += 1
                else:
                    print("SHOULD HAVE REACHED GOAL", (cur_pos[0], cur_pos[2], cur_ori))
                    break

                paths[nr_episodes] = statesPath.copy()
                rewardSum[nr_episodes] += self.highLevel.reward(next_State, r_t-last_t)

                if not first:
                    self.highLevel.updateQTable(cur_State, next_State, highAction, r_t-last_t)
                else:
                    print("THIS SHOULD NEVER HAPPPEN DURING TRAINING")
                    highAction = self.highLevel.chooseAction(next_State)
                    self.highLevel.updateQTable(next_State, next_State, highAction, r_t-last_t)

                avg_lowLvlTimes.append(r_t-last_t)
                last_t = r_t

                # Reset Robot to initial position
                cur_State  = 0
                next_State = 0
                first = True
                statesPath.clear()

                nr_episodes += 1
                if nr_episodes < self.highLevel.episodes/4:
                    self.translation_field.setSFVec3f(init_pos1)
                    self.rotation_field.setSFRotation(init_ori1)
                elif nr_episodes < self.highLevel.episodes/2:
                    self.translation_field.setSFVec3f(init_pos2)
                    self.rotation_field.setSFRotation(init_ori2)
                elif nr_episodes < self.highLevel.episodes*(3/4):
                    self.translation_field.setSFVec3f(init_pos3)
                    self.rotation_field.setSFRotation(init_ori3)
                else:
                    self.translation_field.setSFVec3f(init_pos4)
                    self.rotation_field.setSFRotation(init_ori4)
                self.robot_node.resetPhysics()
                print("EPISODE ", nr_episodes)

            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.05:
                # read Sharp sensors outputs
                dsValues = []
                for i in range(len(self.ds)):
                    dsValues.append(self.ds[i].getValue())
                
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break

            # Get distances of sensors' voltages
            F, FL, L, FR, R, B = [self.frontBrain.sensorVoltageToDistance(dsVal) for dsVal in dsValues]

            if corridor:
                # High Level Training
                r_t = round(self.robot.getTime(), 2)
                # State reached after last highAction (each position has a margin of 3cm since the robot won't always do the same path)
                next_State = self.highLevel.getState((cur_pos[0], cur_pos[2], cur_ori))
                
                # Robot in door entrance and facing door
                if next_State != '':
                    statesPath.append(next_State)
                    rewardSum[nr_episodes] += self.highLevel.reward(next_State, r_t-last_t)

                    # Updates occur only after executing the first highLevel action (so that cur_state and next_state exist)
                    if not first:
                        self.highLevel.updateQTable(cur_State, next_State, highAction, r_t-last_t)
                    
                    # New QLearning cicle starts here
                    cur_State = next_State
                    highAction = self.highLevel.chooseAction(cur_State)
                    first = False        
                    avg_lowLvlTimes.append(r_t-last_t)
                    last_t = r_t

                    if next_State not in visited_states:
                        visited_states[next_State] = 1
                    else:
                        visited_states[next_State] += 1

                    print("DOOR:", self.highLevel.statesNames[next_State], "S:", next_State, "P:", (cur_pos[0], cur_pos[2], cur_ori), "A:", highAction)
                    if highAction == "right":
                        right = True
                        left  = False
                        front = False
                    elif highAction == "left":
                        right = False
                        left  = True
                        front = False
                    elif highAction == "front":
                        right = False
                        left  = False
                        front = True                
                    else:
                        print(highAction)
                        break
                    corridor = False
                    corners = False
                
                # Corners
                elif F < 0.3 and B == 0.3:
                    # Right Corner
                    if FR == 0.3 and R == 0.3:
                        right = True
                        left = False
                        front = False
                        corridor  = False
                        corners = True
                    # Left Corner
                    elif FL == 0.3 and L == 0.3:
                        right = False
                        left = True
                        front = False
                        corridor  = False
                        corners = True
                            
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R < FR and L < FL and F == 0.3 and (left or right or front):
                if not corners:
                    print("IN CORRIDOR " + str((cur_pos[0], cur_pos[2], cur_ori)))
                front    = False
                left     = False
                right    = False
                corridor = True
                corners = False

            # Current state of the robot
            state = self.frontBrain.sensorsToState(dsValues)        
            if left and not right and not front and not corridor:
                action = self.leftBrain.chooseAction(state) # best action in current state

                # Using symmetry
                # rightState = self.rightBrain.symmetricState(state)
                # rightAction = self.rightBrain.chooseAction(rightState)
                # action = self.rightBrain.symmetricAction(rightAction)
                
            elif right and not left and not front and not corridor:
                action = self.rightBrain.chooseAction(state)
            elif (front or corridor) and not right and not left:
                action = self.frontBrain.chooseAction(state)
            else:
                print("ONLY ONE SHOULD BE TRUE F:", front, " R:", right, " L:", left, " C:", corridor)
                end = True
                break
            speeds = self.frontBrain.actionToSpeed(action)   # speed of each motor

            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.1:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break    
            if end or nr_episodes >= self.highLevel.episodes:
                break

        print("SAVING Qtable...")
        self.highLevel.saveQTable(Qtable_name)
        print("Qtable saved!")

        print("SAVING Visited States...")
        json.dump(visited_states, open(visitedStates_name, "w+"))
        print("Visited States saved!")

        print("SAVING Steps per episode...")
        json.dump(paths, open(paths_name, "w+"))
        print("Steps per episode saved!")

        print("SAVING Reward Sum...")
        f = open(rewardSum_name, "w+")
        for r in rewardSum:
            f.write(str(r)+"\n")
        print("Reward Sum saved!")

        print("SAVING Avg Low Level Time...")
        f = open(avg_lowLvl_name, "w+")
        for t in avg_lowLvlTimes:
            f.write(str(t)+"\n")
        print("Avg Low Level Time saved!")
        # Enter here exit cleanup code.
        exit(0)

agentController()