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

        # HighLvl
        first      = True
        last_highState  = None
        next_highState  = None
        next_highAction = None
        last_highAction = None
        last_t = 0
        nr_episodes = 0

        # Select maze
        maze = 2

        # Robot Start Positions
        if maze == 1:
            # MAZE 1
            self.highLvl = highLvlAgent(alpha=0.3, gamma=0.99, episodes=300, epsilon=0.3, statesFile="./mazes/maze1.txt")
            init_pos1 = [0.85,0,0.85]
            init_ori1 = [0,1,0,1.57]
            init_pos2 = [-0.85,0,0.65]
            init_ori2 = [0,1,0,0]
            init_pos3 = [-0.65,0,-0.85]
            init_ori3 = [0,1,0,-1.57]
            init_pos4 = [-0.15,0,-0.55]
            init_ori4 = [0,1,0,-1.57]
            Qtable_name = "QTable_HighLevel_maze1.txt"
            memory_name = "MultiGoal_paths_maze1.txt"
        else:
            # MAZE 2
            self.highLvl = highLvlAgent(alpha=0.3, gamma=0.99, episodes=600, epsilon=0.3, statesFile="./mazes/maze2.txt")
            # Starting from home
            init_pos1 = [0.8,0,1.15]
            init_ori1 = [0,1,0,1.57]
            # Starting from restaurant
            init_pos2 = [-1.15,0,0.6]
            init_ori2 = [0,1,0,0]
            # Starting from college
            init_pos3 = [-0.7,0,-1.15]
            init_ori3 = [0,1,0,-1.57]
            # Starting from library
            init_pos4 = [0.35,0,0]
            init_ori4 = [0,1,0,3.14]
            Qtable_name = "QTable_HighLevel_maze2.txt"
            memory_name = "MultiGoal_paths_maze2.txt"

        # Set robot intial position
        self.translation_field.setSFVec3f(init_pos1)
        self.rotation_field.setSFRotation(init_ori1)
        self.robot_node.resetPhysics()

        # Path memory to be able to go from goal to goal
        paths = dict()
        statesPath = []
        startGoal = 'home1'
        finishGoal = 'gym1' 

        log = open("log.txt", "w+")

        while self.robot.step(self.timestep) != -1:    
            # Obtain robot position each timestep
            cur_pos = self.getCurrentPosition()
            next_highState = self.highLvl.getState(cur_pos, checkingGoal=True)
            
            # Each timestep check if robot reached goal
            if self.highLvl.reachedGoal(next_highState, startGoal):
                r_t = round(self.robot.getTime(), 2)
                goalName = self.highLvl.getGoalName(next_highState)

                if not first:
                    print(last_highState, next_highState, last_highAction)
                    self.highLvl.updateQTable(last_highState, next_highState, last_highAction, r_t-last_t, finishGoal)
                last_t = r_t
                    
                k = str((startGoal, goalName))
                statesPath.append((next_highState, "stop"))

                print("FULL PATH:",statesPath)
                log.write("\nFULL PATH: " + str(statesPath))

                if k not in paths:
                    paths[k] = statesPath.copy()
                else:
                    if len(paths[k]) >= len(statesPath):
                        paths[k] = statesPath.copy()

                # Each goal it reaches start new path
                statesPath.clear()

                if goalName == finishGoal:
                    # Reset Robot to initial position
                    last_highState  = None
                    next_highState  = None
                    last_highAction = None
                    next_highAction = None
                    first = True
    
                    nr_episodes += 1
                    if nr_episodes < self.highLvl.episodes/4:
                        self.translation_field.setSFVec3f(init_pos1)
                        self.rotation_field.setSFRotation(init_ori1)
                        startGoal = "home1"
                    elif nr_episodes < self.highLvl.episodes/2:
                        self.translation_field.setSFVec3f(init_pos2)
                        self.rotation_field.setSFRotation(init_ori2)
                        startGoal = "restaurant1"
                    elif nr_episodes < self.highLvl.episodes*(3/4):
                        self.translation_field.setSFVec3f(init_pos3)
                        self.rotation_field.setSFRotation(init_ori3)
                        startGoal = "college1"
                    else:
                        self.translation_field.setSFVec3f(init_pos4)
                        self.rotation_field.setSFRotation(init_ori4)
                        startGoal = "library2"
                    self.robot_node.resetPhysics()
                    
                    # Update variable of robot current position
                    cur_pos = self.getCurrentPosition()
                    print("EPISODE ", nr_episodes, "\nGoing from", startGoal, "to", finishGoal)
                    log.write("\n\nEPISODE " + str(nr_episodes) + " Going from " + startGoal + " to " + finishGoal)
                # Found a goal while going from startGoal to finishGoal
                # Split path between startGoal -> middleGoal -> finishGoal
                else:
                    print("From", startGoal, "found", goalName, "while training to reach", finishGoal, "\nSaving that info...")                    
                    log.write("\nFrom " + startGoal + " found " + goalName + " while training to reach " + finishGoal)                    
                    # startGoal will change to middleGoal and start a new path
                    startGoal = goalName

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
            
            # Get high level action for current position if any
            next_highState, next_highAction = self.getHighLvlAction(cur_pos)
    
            if next_highState != None:
                stateName = self.highLvl.getDoorName(next_highState)
                print("StateName:", stateName, "S:", next_highState, "A:", next_highAction)
                log.write("\nStateName: " + stateName + " S: " + next_highState + " A: " + next_highAction)
                
                r_t = round(self.robot.getTime(), 2)
                # Every state the agent reaches save pair (state, action) that led to it
                statesPath.append((next_highState, next_highAction))

                # Updates occur only after executing the first highLevel action (so that cur_state and next_state exist)
                if not first and last_highAction != None:
                    self.highLvl.updateQTable(last_highState, next_highState, last_highAction, r_t-last_t, finishGoal)
                
                # New QLearning cicle starts here
                last_highState = next_highState
                last_highAction = next_highAction
                first = False        
                last_t = r_t

            if next_highAction == "back":
                self.turn_on_axis(180,1)
            else:
                # Get low level action according to high level one
                lowAction = self.getLowLvlAction(dsValues, next_highAction, cur_pos)                
            
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
            if end or nr_episodes >= self.highLvl.episodes:
                break

        print("SAVING Qtable...")
        self.highLvl.saveQTable(Qtable_name)
        print("Qtable saved!")

        print("Saving paths on memory...")
        json.dump(paths, open(memory_name, "w+"))
        print("Memory saved!")

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
            if state != None:
                return (state, self.highLvl.chooseAction(state))
        return (None, None)

    # From High Level Action and sensors' states define Low Level Action 
    # Go forward on doors or corridor, turn left or right on corners and doors
    def getLowLvlAction(self, dsValues, highAction, cur_pos):
        # Convert sensors' voltages to distances
        F, FL, L, FR, R, B = [self.frontBrain.sensorVoltageToDistance(dsVal) for dsVal in dsValues]

        if self.corridor:                
            # Define Low Level Action based on High Level
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
            

agentController()