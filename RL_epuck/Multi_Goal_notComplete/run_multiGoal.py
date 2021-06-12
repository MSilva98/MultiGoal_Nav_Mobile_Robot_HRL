"""train_multiGoal controller."""
# python script to control the robot with a QTable after training

# You may need to import some classes of the controller module. Ex:
from os import path, stat
from controller import Supervisor
import numpy as np
from lowLevel import Agent as lowLvlAgent
from highLevel import Agent as highLvlAgent
import json
import math

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
        self.leftBrain  = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_left_all.txt")  # QTable to go left

        # define the time step of the current world.
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
        end = False
        
        # Low Level
        self.front    = False
        self.right    = False
        self.left     = False
        self.corridor = True

        # Select maze
        maze = 2

        # Robot Start Positions
        if maze == 1:
            # MAZE 1
            self.highLvl = highLvlAgent(epsilon=0, statesFile="./mazes/maze1.txt", paths="MultiGoal_paths_maze1.txt")
        else:
            # MAZE 2
            self.highLvl = highLvlAgent(epsilon=0, statesFile="./mazes/maze2.txt", paths="MultiGoal_paths_maze2.txt")

        # Set robot intial position
        # TODO: SELECT RANDOM POSITION FROM GOALS HERE
        startGoal = self.highLvl.getRandomGoal()
        finishGoal = self.highLvl.getRandomGoal(startGoal)
        startState = self.highLvl.getGoalState(startGoal)
        init_pos, init_ori = self.convertStateToWebots(startState)
        print(init_pos)
        self.translation_field.setSFVec3f(init_pos)
        self.rotation_field.setSFRotation(init_ori)
        self.robot_node.resetPhysics()

        # Path memory to be able to go from goal to goal
        # graph = self.createGoalsGraph(self.highLvl.paths)
        # json.dump(graph, open("test_graph.txt", "w+"))

        stateActionPairs = self.getPath(self.highLvl.paths, startGoal, finishGoal)
        print(stateActionPairs)
        
        exit(0)
        while self.robot.step(self.timestep) != -1:    
            # Obtain robot position each timestep
            cur_pos = self.getCurrentPosition()
            
            # Each timestep check if robot reached goal
            if self.highLvl.reachedGoal(cur_pos, finishGoal):
                startGoal = finishGoal
                finishGoal = self.highLvl.getRandomGoal(startGoal)
                stateActionPairs = self.getPath(self.highLvl.paths, startGoal, finishGoal)
                print("Going from", startGoal, "to", finishGoal)
                
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
            
            highState = self.highLvl.getState(cur_pos)
            highAction = stateActionPairs[highState]

            # # Get high level action for current position if any
            # _, highAction = self.getHighLvlAction(cur_pos)
       
            if highAction == "back":
                self.turn_on_axis(180,1)
            else:
                # Get low level action according to high level one
                lowAction = self.getLowLvlAction(dsValues, highAction, cur_pos)                
            
                # When more than one action is True
                if lowAction == None:
                    end = True
                    break
            
                # Speed applied to each motor
                speeds = self.frontBrain.actionToSpeed(lowAction)   
                t = self.robot.getTime()
                while self.robot.getTime() - t < 0.1:
                    # Take action
                    self.leftMotor.setVelocity(speeds[0])
                    self.rightMotor.setVelocity(speeds[1])
                    # controller termination
                    if self.robot.step(self.timestep) == -1:
                        end = True
                        break   

            # controller termination 
            if end:
                break

        # Enter here exit cleanup code.
        exit(0)

    # Perform a rotation by itself of angle degrees
    # Direction = 1 -> clockwise
    # Direction = -1 -> counterclockwise
    def turn_on_axis(self, angle, direction):
        # E-Puck properties
        axelLength = 0.052
        wheelRadius = 0.0205 
        wheelSpeed = 2
        angle_rads = angle*math.pi/180

        delta = angle_rads*axelLength/(2*wheelRadius*wheelSpeed)
        t = self.robot.getTime()
        while self.robot.getTime() - t <= delta:
            # Take action
            self.leftMotor.setVelocity(wheelSpeed*direction)
            self.rightMotor.setVelocity(-wheelSpeed*direction)
            # controller termination
            if self.robot.step(self.timestep) == -1:
                exit()

    # Check if robot reached the same door from last state
    def sameDoor(self, lastState, highState):
        # Door Names are all composed by two letters and door number
        # E.g.: For door 4 one has: fr4, fl4 and lr4
        # If FLR doors were implemented this had to be changed
        lastDoor = self.highLvl.getDoorName(lastState)
        curDoor = self.highLvl.getDoorName(highState)
        return lastDoor[2:] == curDoor[2:]

    # Get current robot position (x, z, theta)
    def getCurrentPosition(self):
        cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
        cur_ori = round(self.rotation_field.getSFRotation()[3],4)
        return (cur_pos[0], cur_pos[2], cur_ori)

    # Get High Level Action for current robot position if reached door
    def getHighLvlAction(self, cur_pos):
        if self.corridor:
            # High Level State
            state = self.highLvl.getState(cur_pos)
            if state != '':
                doorName = self.highLvl.getDoorName(state)
                print("DOOR:", doorName, "S:", state)
                return (state, self.highLvl.chooseAction(state))
        return (None, None)

    # From High Level Action and sensors' states define Low Level Action 
    # Go forward on doors or corridor, turn left or right on corners and doors
    def getLowLvlAction(self, dsValues, highAction, cur_pos):
        # Convert sensors' voltages to distances
        F, FL, L, FR, R, B = [self.frontBrain.sensorVoltageToDistance(dsVal) for dsVal in dsValues]

        if self.corridor:
            # Robot in door entrance
            if highAction != None:
                print("P:", cur_pos, "A:", highAction)
                
            # Define Low Level Action
            if highAction == "right":
                self.right    = True
                self.left     = False
                self.front    = False
                self.corridor = False
            elif highAction == "left":
                self.right    = False
                self.left     = True
                self.front    = False
                self.corridor = False
            elif highAction == "front":
                self.right    = False
                self.left     = False
                self.front    = True                
                self.corridor = False
                
            # Corners
            elif F < 0.3 and B == 0.3:
                # Right Corner
                if FR == 0.3 and R == 0.3:
                    self.right    = True
                    self.left     = False
                    self.front    = False
                    self.corridor = False
                # Left Corner
                elif FL == 0.3 and L == 0.3:
                    self.right    = False
                    self.left     = True
                    self.front    = False
                    self.corridor = False
                        
        # Corridor
        elif FL < 0.3 and FR < 0.3 and R < FR and L < FL and F == 0.3 and (self.left or self.right or self.front):
            self.right    = False
            self.left     = False
            self.front    = False
            self.corridor = True

        # Current state of the robot
        state = self.frontBrain.sensorsToState(dsValues)        
        if self.left and not self.right and not self.front and not self.corridor:
            action = self.leftBrain.chooseAction(state)       
        elif self.right and not self.left and not self.front and not self.corridor:
            action = self.rightBrain.chooseAction(state)
        elif (self.front or self.corridor) and not self.right and not self.left:
            action = self.frontBrain.chooseAction(state)
        else:
            print("ONLY ONE SHOULD BE TRUE F:", self.front, " R:", self.right, " L:", self.left, " C:", self.corridor)
            return None
        return action

    def createGoalsGraph(self, paths):
        # From paths file create a graph between connected paths
        graph = dict()
        for goalTup in paths:
            start, finish = goalTup.replace('(','').replace(')','').replace(' ', '').replace("'", '').split(',')
            if start[:-1] != finish[:-1]:
                print(start, finish)
                startState = self.highLvl.getGoalState(start)
                finishState = self.highLvl.getGoalState(finish)
                doors = paths[goalTup]
                
                if startState not in graph:
                    graph[startState] = [doors[0]]
                else:
                    graph[startState].append(doors[0])

                for i in range(len(doors)-1):
                    if doors[i][0] not in graph:
                        graph[doors[i][0]] = [doors[i+1]]
                    else:
                        graph[doors[i][0]].append(doors[i+1])

                if doors[-1][0] not in graph:
                    graph[doors[-1][0]] = [finishState]
                else:
                    graph[doors[-1][0]].append(finishState)
        return graph

    def getPath(self, paths, startGoal, finishGoal):
        # Return dictionary {'state': 'action'} with all necessary actions to 
        # go from startGoal to finishGoal
        key = str((startGoal, finishGoal))
        print(key)
        if key in paths:
            return paths[key]
        return None
        
    def convertStateToWebots(self, state):
        # Convert state string to webots lists for position and orientation
        x,z,theta = [float(v) for v in state.replace('(','').replace(')','').replace(' ', '').split(',')]
        pos = [x, 0, z]
        ori = [0,1,0,theta]    
        return pos, ori

agentController()