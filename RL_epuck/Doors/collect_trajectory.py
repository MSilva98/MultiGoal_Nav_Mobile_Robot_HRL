"""collect_trajectory controller."""
# python script to obtain the robot trajectory going through doors

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
        # create supervisor instance (Set of fucorrected_v5_6s_4e_5000h_oncenctions available for each robot node)
        self.robot = Supervisor()
        # node to use Supervisor functions
        self.robot_node = self.robot.getFromDef("epuck")
        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        self.wall_node = self.robot.getFromDef("wallDoors")
        self.wall_pos = self.wall_node.getField("translation")
        self.wall_rot = self.wall_node.getField("rotation")
        # Robot reinforcement learning brain
        # NOTE: epsilon MUST be 0 to disable random actions

        # BEST FR FL
        # self.leftBrain  = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_FL_Left_z0_x20_sym.txt")   # QTable to go left on FL doors
        # self.rightBrain = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_FR_Right_z0_x20.txt")  # QTable to go right on FR doors

        # BEST LR
        # self.leftBrain  = Agent(epsilon=0, Qtable="./lr/QTable_v5_17500h_LR_Left.txt")   # QTable to go left on FL doors
        # self.rightBrain = Agent(epsilon=0, Qtable="./lr/QTable_v5_17500h_LR_Right.txt")  # QTable to go right on FR doors

        # self.leftBrain  = Agent(epsilon=0, Qtable="./lr/QTable_v5_17500h_FL_Left.txt")   # QTable to go left on FL doors
        # self.rightBrain = Agent(epsilon=0, Qtable="./lr/QTable_v5_17500h_FR_Right.txt")  # QTable to go right on FR doors

        # self.frontBrain = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Front.txt") # QTable for corridor and to go forward
        # self.leftBrain  = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Left_Door.txt")   # QTable to go left on FL doors
        # self.rightBrain = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Right_Door.txt")  # QTable to go right on FR doors

        # self.leftBrain  = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_LR_Left_sym.txt")   # QTable to go left on FL doors
        # self.rightBrain  = Agent(epsilon=0, Qtable="./v5/QTable_v5_13000h_corrected_LR_Right.txt")   # QTable to go left on FL doors
                
        self.frontBrain    = Agent(epsilon=0, Qtable="./v52/QTable_v5_17500h_FR_Front.txt")        # QTable for corridor and to go forward        
        # self.leftFL_Brain  = Agent(epsilon=0, Qtable="./v52/QTable_v5_17500h_FL_Left.txt")      # QTable to go left on FL and LR Doors
        # self.rightFR_Brain = Agent(epsilon=0, Qtable="./v52/QTable_v5_17500h_FR_Right.txt")     # QTable to go right on FR and LR Doors
        # self.leftLR_Brain  = Agent(epsilon=0, Qtable="./v52/QTable_v5_17500h_LR_Left_1.txt")      # QTable to go right on FR and LR Doors
        # self.rightLR_Brain = Agent(epsilon=0, Qtable="./v52/QTable_v5_17500h_LR_Right_1.txt")     # QTable to go right on FR and LR Doors
        
        self.leftBrain  = Agent(epsilon=0, Qtable="./v52/QTable_left.txt")      # QTable to go left on FL and LR Doors
        self.rightBrain = Agent(epsilon=0, Qtable="./v52/QTable_right.txt")     # QTable to go right on FR and LR Doors
        
        # self.leftFL_Brain  = Agent(epsilon=0, Qtable="./v52/QTable_left.txt")      # QTable to go left on FL and LR Doors
        # self.rightFR_Brain = Agent(epsilon=0, Qtable="./v52/QTable_right.txt")     # QTable to go right on FR and LR Doors
        # self.leftLR_Brain  = Agent(epsilon=0, Qtable="./v52/QTable_left.txt")      # QTable to go right on FR and LR Doors
        # self.rightLR_Brain = Agent(epsilon=0, Qtable="./v52/QTable_right.txt")     # QTable to go right on FR and LR Doors
        
        self.leftC_Brain   = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Left_Corner.txt")  # QTable to go left on Corner
        self.rightC_Brain  = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Right_Corner.txt") # QTable to go right on Corner

        
        # self.frontBrain    = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Front.txt")        # QTable for corridor and to go forward
        # self.leftFL_Brain  = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_FL_Left.txt")      # QTable to go left on FL and LR Doors
        # self.rightFR_Brain = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_FR_Right.txt")     # QTable to go right on FR and LR Doors
        # self.leftLR_Brain  = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_LR_Left.txt")      # QTable to go right on FR and LR Doors
        # self.rightLR_Brain = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_LR_Right.txt")     # QTable to go right on FR and LR Doors
        # self.leftC_Brain   = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Left_Corner.txt")  # QTable to go left on Corner
        # self.rightC_Brain  = Agent(epsilon=0, Qtable="../LowLevelCombined/QTable_Right_Corner.txt") # QTable to go right on Corner


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
        all_pos_right = []
        all_pos_left = []
        all_pos = []
        visited_states = dict()
        last_t = 0
        left   = False
        right  = False
        leftC    = False
        rightC   = False
        front    = False
        corridor = True

        all_together = True
        rightDoor = True
        lrDoor = True

        if not all_together:
            if lrDoor:
                self.wall_pos.setSFVec3f([0,0,-0.155])
                self.wall_rot.setSFRotation([0,1,0,0])
                self.wall_node.resetPhysics()
            else:
                self.wall_pos.setSFVec3f([-0.155,0,0])
                self.wall_rot.setSFRotation([0,1,0,-1.5708])
                self.wall_node.resetPhysics()

        while self.robot.step(self.timestep) != -1:  
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]

            t = self.robot.getTime()
            if all_together:
                if round(t,0) - last_t >= 1:
                    last_t = round(t,0)
                    all_pos.append((cur_pos[0], cur_pos[2]))
            else:
                # Everytime the robot reaches the corridor after the door its position is reseted
                if (cur_pos[0] > 0.5 or cur_pos[0] < -0.5) and lrDoor:
                    if last_t < 7200:
                        all_pos_right.append((0,0))
                        rightDoor = True
                    else:
                        all_pos_left.append((0,0))
                        rightDoor = False
                    self.translation_field.setSFVec3f([0.0,0,0.5])
                    self.rotation_field.setSFRotation([0,1,0,0])
                    self.robot_node.resetPhysics()
                elif (cur_pos[0] > 0.40 or cur_pos[2] < -0.20) and last_t < 7200 and not lrDoor:
                    print("NOT LR DOOR")
                    all_pos_right.append((0,0)) # "Point" just to separate each path
                    rightDoor = True
                    self.translation_field.setSFVec3f([0,0,0.4])
                    self.rotation_field.setSFRotation([0,1,0,0])
                    self.robot_node.resetPhysics()
                elif (cur_pos[0] > 0.40 or cur_pos[2] > 0.20) and last_t >= 7200 and not lrDoor:
                    print("NOT LR DOOR")
                    all_pos_left.append((0,0)) # "Point" just to separate each path
                    rightDoor = False
                    self.translation_field.setSFVec3f([0,0,-0.4])
                    self.rotation_field.setSFRotation([0,1,0,3.14])
                    self.robot_node.resetPhysics()
                elif round(t,0) - last_t >= 1:
                    last_t = round(t,0)
                    if rightDoor:
                        all_pos_right.append((cur_pos[0], cur_pos[2]))
                    else:
                        all_pos_left.append((cur_pos[0], cur_pos[2]))

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
                if all_together:
                    actDoor = np.random.choice(["left", "right"])
                else:
                    if last_t < 7200:
                        actDoor = "right"
                    else:
                        actDoor = "left"
                if actDoor == "right":
                    print("ROBOT REACHED LR DOOR AND GOES RIGHT")
                    right = True
                    left  = False
                else:
                    print("ROBOT REACHED LR DOOR AND GOES LEFT")
                    right = False
                    left  = True
                front     = False
                corridor  = False
                leftC     = False
                rightC    = False
                
            # Front Right door/Corridor
            elif FR == 0.3 and R == 0.3 and B == 0.3:
                if corridor:
                    # Front Right Door
                    if F == 0.3:
                        if all_together:
                            actDoor = np.random.choice(["front", "right"])
                        else:
                            actDoor = "right"
                        if actDoor == "right":
                            print("ROBOT REACHED FR DOOR AND GOES RIGHT")
                            right = True
                            front = False
                        elif actDoor == "front":
                            print("ROBOT REACHED FR DOOR AND GOES FORWARD")
                            front = True
                            right = False
                        rightC    = False
                    # Corner to Right
                    else:
                        rightC = True
                        front  = False
                        right  = False
                    corridor   = False
                    leftC      = False
                    left       = False

            # Front Left door/Corridor
            elif FL == 0.3 and L == 0.3 and B == 0.3:
                if corridor:
                    # Front Left Door
                    if F == 0.3:
                        if all_together:
                            actDoor = np.random.choice(["front", "left"])
                        else:
                            actDoor = "left"
                        if actDoor == "left":
                            print("ROBOT REACHED FL DOOR AND GOES LEFT")
                            left = True
                            front  = False
                        elif actDoor == "front":
                            print("ROBOT REACHED FL DOOR AND GOES FORWARD")
                            front  = True
                            left = False
                    # Left Corner
                    else:
                        leftC = True
                        front = False
                        left = False
                    corridor  = False    
                    right   = False
                    rightC    = False
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R <= 0.2 and L <= 0.2 and B == 0.3 and F == 0.3 and (left or right or leftC or rightC or front):
                print("ROBOT EXITS DOOR/CORNER AND IS ON CORRIDOR")
                front    = False
                left   = False
                right  = False
                leftC    = False
                rightC   = False
                corridor = True

            state = self.frontBrain.sensorsToState(dsValues, False) # next state - state after action                                            
            if left and not right and not rightC and not leftC and not front:
                print("Left DOOR")
                action = self.leftBrain.chooseAction(state) # best action in current state
            elif right and not left and not rightC and not leftC and not front:
                print("Right DOOR")
                action = self.rightBrain.chooseAction(state)
            elif leftC and not rightC and not right and not left and not front:
                print("LEFT CORNER")
                action = self.leftC_Brain.chooseAction(state) # best action in current state
            elif rightC and not leftC and not right and not left and not front:
                print("RIGHT CORNER")
                action = self.rightC_Brain.chooseAction(state)
            elif (front or corridor) and not right and not left and not rightC and not leftC:
                action = self.frontBrain.chooseAction(state)
            else:
                print("ONLY ONE SHOULD BE TRUE F:", front, " Right:", right, " Left:", left, " rC:", rightC, " lC:", leftC, " C:", corridor)
                end = True
                break

            speeds = self.frontBrain.actionToSpeed(action)   # speed of each motor
            print("State: ", state, "Action: ", action)

            t = self.robot.getTime()
            while self.robot.getTime() - t < 0.1:
                # Take action
                self.leftMotor.setVelocity(speeds[0])
                self.rightMotor.setVelocity(speeds[1])
                # controller termination
                if self.robot.step(self.timestep) == -1:
                    end = True
                    break

            # End or 2h passed
            if end or round(t,0) >= 14400:
                break
        
        if all_together:
            print("SAVING TRAJECTORY...")
            with open("doors_trajectory.txt", "w+") as f:
                for p in all_pos:
                    f.write(str(p) + "\n")
                print("TRAJECTORY SAVED!")
        else:
            if lrDoor:
                print("SAVING TABLES...")
                with open("door_right_lr_fr.txt", "w+") as f:
                    for p in all_pos_right:
                        f.write(str(p) + "\n")
                print("DOOR RIGHT SAVED!")

                with open("door_left_lr_fl.txt", "w+") as f:
                    for p in all_pos_left:
                        f.write(str(p) + "\n")
                print("DOOR LEFT SAVED!")
            else:
                print("SAVING TABLES...")
                with open("door_right_fr_lr.txt", "w+") as f:
                    for p in all_pos_right:
                        f.write(str(p) + "\n")
                print("DOOR RIGHT SAVED!")

                with open("door_left_fl_lr.txt", "w+") as f:
                    for p in all_pos_left:
                        f.write(str(p) + "\n")
                print("DOOR LEFT SAVED!")

        # with open("visited_states.txt", "w+") as f:
        #     json.dump(visited_states, f)
        # print("VISITED_STATES SAVED!")

        # Enter here exit cleanup code.
        exit(0)

agentController()