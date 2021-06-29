"""run_Agent controller."""
# python script to control the robot with a QTable after training

# You may need to import some classes of the controller module. Ex:
from controller import Supervisor
import numpy as np
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

        # Dynamic Learning
        lastState = ''
        lastAction = ''
        last_t = 0

        # Select maze
        maze = 2
        # Robot Start Positions
        if maze == 1:
            # MAZE 1
            # High Level brain
            self.highLvl = highLvlAgent(epsilon=0, statesFile="./mazes/maze1.txt", Qtable="QTable_HighLevel_maze1.txt")
            init_pos1 = [0.85,0,0.85]
            init_ori1 = [0,1,0,1.57]
            init_pos2 = [-0.85,0,0.65]
            init_ori2 = [0,1,0,0]
            init_pos3 = [-0.65,0,-0.85]
            init_ori3 = [0,1,0,-1.57]
            init_pos4 = [-0.15,0,-0.55]
            init_ori4 = [0,1,0,-1.57]
            Qtable_name = "QTable_HighLevel_maze1_updated.txt"
            memory_name = "maze1_memory.txt"
        else:
            # MAZE 2
            # High Level brain
            self.highLvl = highLvlAgent(epsilon=0, statesFile="./mazes/maze2.txt", Qtable="QTable_HighLevel_maze2.txt")
            init_pos1 = [0.8,0,1.15]
            init_ori1 = [0,1,0,1.57]
            init_pos2 = [-1.15,0,0.6]
            init_ori2 = [0,1,0,0]
            init_pos3 = [-0.7,0,-1.15]
            init_ori3 = [0,1,0,-1.57]
            init_pos4 = [1.15,0,-0.8]
            init_ori4 = [0,1,0,3.14]
            Qtable_name = "QTable_HighLevel_maze2_updated_2.txt"
            memory_name = "maze2_memory_2.txt"

        # Set robot intial position
        init_pos = 0
        self.translation_field.setSFVec3f(init_pos3)
        self.rotation_field.setSFRotation(init_ori3)
        self.robot_node.resetPhysics()

        while self.robot.step(self.timestep) != -1:    
            # Obtain robot position each timestep
            cur_pos = self.getCurrentPosition()
            
            # Each timestep check if robot reached goal
            if self.highLvl.reachedGoal(cur_pos):
                init_pos += 1

                # Demonstration purpose: Use only position 3
                self.translation_field.setSFVec3f(init_pos3)
                self.rotation_field.setSFRotation(init_ori3)
                self.robot_node.resetPhysics()
                print("Start Pos ", init_pos)

                # RESET NECESSARY VARIABLES
                lastAction = ''
                lastState  = ''

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
            highState, highAction = self.getHighLvlAction(cur_pos, 0.2)
    
            if highState != None:
                r_t = round(self.robot.getTime(), 2)
                if lastState != '':    
                    doorName = self.highLvl.statesNames[lastState]
                    # Robot performed action and reached same door
                    if self.sameDoor(lastState, highState):
                        if doorName not in self.highLvl.memoryDoors:
                            self.highLvl.memoryDoors[doorName] = [lastAction]
                        else:
                            if lastAction not in self.highLvl.memoryDoors[doorName]:
                                self.highLvl.memoryDoors[doorName].append(lastAction)
                        print("BACK ON SAME DOOR\nUPDATING QTABLE FOR", doorName)
                        self.highLvl.updateQTable(lastState, highState, lastAction, r_t-last_t)
                    
                    # Robot performed action that should lead to same door but reached different door -> this means new path was found
                    # Went through a path that was blocked before
                    elif doorName in self.highLvl.memoryDoors and lastAction in self.highLvl.memoryDoors[doorName] and not self.sameDoor(lastState, highState):
                        print("NEW PATH\nUPDATING QTABLE FOR", doorName)
                        self.highLvl.updateQTable(lastState, highState, lastAction, r_t-last_t)
                        # When the action that leads to the new path is the best one remove the door from the memory
                        # This is ok because in order to the action be added to the memory it had to be the best one previous to the blocking
                        if self.highLvl.QTable[lastState][lastAction] == self.highLvl.maxQ(lastState):
                            if len(self.highLvl.memoryDoors[doorName]) <= 1:
                                self.highLvl.memoryDoors.pop(doorName, None)
                            else:
                                self.highLvl.memoryDoors[doorName].remove(lastAction)

                lastState = highState
                lastAction = highAction
                last_t = r_t
                    
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

        # print("SAVING Qtable...")
        # self.highLvl.saveQTable(Qtable_name)
        # print("Qtable saved!")

        # print("Saving robot memory from doors...")
        # self.highLvl.saveMemory(memory_name)
        # print("Memory saved!")

        # Enter here exit cleanup code.
        exit(0)

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
    def getHighLvlAction(self, cur_pos, epsilon):
        if self.corridor:
            # High Level State
            state = self.highLvl.getState(cur_pos)
            if state != '':
                doorName = self.highLvl.getDoorName(state)
                # Reached door in the memory -> means it has been blocked
                if doorName in self.highLvl.memoryDoors:
                    # Enable exploration probability
                    self.highLvl.epsilon = epsilon
                else:
                    # All other doors disable exploration
                    self.highLvl.epsilon = 0
                print("DOOR:", doorName, "S:", state, "Epsilon:", self.highLvl.epsilon)
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
            

agentController()