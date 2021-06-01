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
        # Symetry can be used instead (which is better computationally?)
        self.leftBrain  = lowLvlAgent(epsilon=0, Qtable="./LowLevelTables/QTable_left_all.txt")  # QTable to go left

        # self.front = json.load(open("./LowLevelTables/QTable_corridor_trimmed.txt", "r"))  # QTable for corridor and to go forward
        # self.rightBrain = json.load(open("./LowLevelTables/QTable_right_all_trimmed.txt", "r")) # QTable to go right
        # self.leftBrain  = json.load(open("./LowLevelTables/QTable_left_all_trimmed.txt", "r"))  # QTable to go left

        self.highLevel = highLvlAgent(epsilon=0, statesFile="maze2.txt", Qtable="QTable_HighLevel_maze2.txt") # QTable to choose actions on doors
       
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

        # HighLvl
        highState = 0
        highAction = ''
        init_pos = 1
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
        self.translation_field.setSFVec3f(init_pos3)
        self.rotation_field.setSFRotation(init_ori3)
        self.robot_node.resetPhysics()

        while self.robot.step(self.timestep) != -1:    
            # Current position of robot each timestep
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
            cur_ori = round(self.rotation_field.getSFRotation()[3],4)
           
            # Each timestep check if robot reached goal
            if self.highLevel.reachedGoal((cur_pos[0], cur_pos[2])):
                init_pos += 1
                # RESET ROBOT
                if init_pos == 2:
                    self.translation_field.setSFVec3f(init_pos2)
                    self.rotation_field.setSFRotation(init_ori2)
                elif init_pos == 3:
                    self.translation_field.setSFVec3f(init_pos3)
                    self.rotation_field.setSFRotation(init_ori3)
                elif init_pos == 4:
                    self.translation_field.setSFVec3f(init_pos4)
                    self.rotation_field.setSFRotation(init_ori4)
                else:
                    end = True
                    break
                self.robot_node.resetPhysics()
                print("Start Pos ", init_pos)

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
                
                # High Level Action Choose
                highState = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                if highState == '':
                    print("LR: ", cur_pos[0], cur_pos[2], cur_ori)
                    break
                highAction = self.highLevel.chooseAction(highState)
                
                if highAction == "right":
                    print("ROBOT REACHED LR DOOR AND GOES RIGHT " + str((cur_pos[0], cur_pos[2])))
                    right = True
                    left  = False
                elif highAction == "left":
                    print("ROBOT REACHED LR DOOR AND GOES LEFT " + str((cur_pos[0], cur_pos[2])))
                    right = False
                    left  = True
                front     = False
                corridor  = False
                
            # Front Right door/Corridor
            elif FR == 0.3 and R == 0.3 and B == 0.3:
                if corridor:
                    # Front Right Door
                    if F == 0.3:
                        # highAction = np.random.choice(["front", "right"])
                        
                        # High Level Action Choose
                        highState = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                        if highState == '':
                            print("FR ", cur_pos[0], cur_pos[2], cur_ori)
                            break
                        highAction = self.highLevel.chooseAction(highState)

                        if highAction == "right":
                            print("ROBOT REACHED FR DOOR AND GOES RIGHT " + str((cur_pos[0], cur_pos[2])))
                            right = True
                            front = False
                        elif highAction == "front":
                            print("ROBOT REACHED FR DOOR AND GOES FORWARD " + str((cur_pos[0], cur_pos[2])))
                            front = True
                            right = False
                    # Corner to Right
                    else:
                        # print("ROBOT REACHED RIGHT CORNER")
                        right = True
                        front = False
                    corridor  = False

            # Front Left door/Corridor
            elif FL == 0.3 and L == 0.3 and B == 0.3:
                if corridor:
                    # Front Left Door
                    if F == 0.3:  
                        # highAction = np.random.choice(["front", "left"])
                        
                        # High Level Action Choose
                        highState = self.highLevel.getState((cur_pos[0], cur_pos[2]))
                        if highState == '':
                            print("FL ", cur_pos[0], cur_pos[2], cur_ori)
                            break
                        highAction = self.highLevel.chooseAction(highState)

                        if highAction == "left":
                            print("ROBOT REACHED FL DOOR AND GOES LEFT " + str((cur_pos[0], cur_pos[2])))
                            left  = True
                            front = False
                        elif highAction == "front":
                            print("ROBOT REACHED FL DOOR AND GOES FORWARD " + str((cur_pos[0], cur_pos[2])))
                            front = True
                            left  = False
                    # Left Corner
                    else:
                        # print("ROBOT REACHED LEFT CORNER")
                        left  = True
                        front = False
                    corridor  = False    
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R <= 0.2 and L <= 0.2 and R < FR and L < FL and B == 0.3 and F == 0.3 and (left or right or front):
                print("ROBOT EXITS DOOR/CORNER AND IS ON CORRIDOR")
                front    = False
                left     = False
                right    = False
                corridor = True

            # Current state of the robot
            state = self.frontBrain.sensorsToState(dsValues)        
            if left and not right and not front and not corridor:
                # print("Turn Left")
                action = self.leftBrain.chooseAction(state)
                # action = self.leftBrain[state] # best action in current state

                # Using symmetry
                # rightState = self.rightBrain.symmetricState(state)
                # rightAction = self.rightBrain.chooseAction(rightState)
                # action = self.rightBrain.symmetricAction(rightAction)
                
            elif right and not left and not front and not corridor:
                # print("Turn Right")
                action = self.rightBrain.chooseAction(state)
                # action = self.rightBrain[state]
            elif (front or corridor) and not right and not left:
                # if front:
                    # print("Go Forward")
                action = self.frontBrain.chooseAction(state)
                # action = self.front[state]
            else:
                print("ONLY ONE SHOULD BE TRUE F:", front, " R:", right, " L:", left, " C:", corridor)
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
            if end:
                break

        # Enter here exit cleanup code.
        exit(0)

agentController()