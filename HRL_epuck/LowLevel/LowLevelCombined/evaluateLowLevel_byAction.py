"""run_Agent controller."""
# python script to control the robot with a QTable after training

# You may need to import some classes of the controller module. Ex:
from controller import Supervisor
import numpy as np
from agent import Agent as lowLvlAgent
import math
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
        self.frontBrain = lowLvlAgent(epsilon=0, Qtable="QTable_corridor.txt")  # QTable for corridor and to go forward
        self.rightBrain = lowLvlAgent(epsilon=0, Qtable="QTable_right_all.txt") # QTable to go right
        self.leftBrain  = lowLvlAgent(epsilon=0, Qtable="QTable_left_all.txt")  # QTable to go left

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
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        end      = False
        action   = ''
        left     = False
        right    = False
        front    = False
        corridor = True

        statesF = json.load(open("maze8.txt", "r"))
        self.doors = {}
        for state in statesF:
            if statesF[state][:2] == "fr":
                self.doors[state] = ["front", "right"]
            elif statesF[state][:2] == "fl":
                self.doors[state] = ["front", "left"]
            elif statesF[state][:2] == "lr":
                self.doors[state] = ["left", "right"]
        
        # Trajectory collection
        trajectory = []
        last_t = 0

        laps = 1
        last_x = 0
        errors = 0
        
        # doorAction = "front"
        # doorAction = "left"
        doorAction = "right"
        name = "trajectory_"+doorAction+".txt"

        if doorAction == "front":
            self.translation_field.setSFVec3f([-0.11,0,-0.85])
            self.rotation_field.setSFRotation([0,1,0,1.57])
            self.robot_node.resetPhysics()
        else: 
            self.translation_field.setSFVec3f([-0.11,0,0])
            self.rotation_field.setSFRotation([0,1,0,1.57])
            self.robot_node.resetPhysics()

        while self.robot.step(self.timestep) != -1:    
            # Current position of robot each timestep
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
            cur_ori = round(self.rotation_field.getSFRotation()[3],4)
            
            if doorAction == "left" and (cur_pos[2] <= -0.25 or (abs(cur_pos[2]) <= 0.15 and cur_ori < -1.0472 and cur_ori > -2.0944)):
                    print("################################################")
                    print("################################################")
                    print("############### ERROR GOING LEFT ###############")
                    print("################################################")
                    print("################################################")
                    print("AT POS:", cur_pos, cur_ori)
                    errors += 1
                    self.translation_field.setSFVec3f([-0.11,0,0])
                    self.rotation_field.setSFRotation([0,1,0,1.57])
                    self.robot_node.resetPhysics()
                    trajectory.append((-5,-5))
                    # Update robot position
                    cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
                    cur_ori = round(self.rotation_field.getSFRotation()[3],4)

            elif doorAction == "right" and (cur_pos[2] >= 0.25 or (abs(cur_pos[2]) <= 0.15 and cur_ori < -1.0472 and cur_ori > -2.0944)):
                    print("#################################################")
                    print("#################################################")
                    print("############### ERROR GOING RIGHT ###############")
                    print("#################################################")
                    print("#################################################")
                    errors += 1
                    self.translation_field.setSFVec3f([-0.11,0,0])
                    self.rotation_field.setSFRotation([0,1,0,1.57])
                    self.robot_node.resetPhysics()
                    trajectory.append((-5,-5))
                    # Update robot position
                    cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
                    cur_ori = round(self.rotation_field.getSFRotation()[3],4)

            elif doorAction == "front" and abs(cur_pos[0]) < 0.6 and abs(cur_pos[2]) < 0.15:
                print("#################################################")
                print("#################################################")
                print("############### ERROR GOING FRONT ###############")
                print("#################################################")
                print("#################################################")
                errors += 1
                self.translation_field.setSFVec3f([-0.11,0,-0.85])
                self.rotation_field.setSFRotation([0,1,0,1.57])
                self.robot_node.resetPhysics()
                trajectory.append((-5,-5))
                # Update robot position
                cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
                cur_ori = round(self.rotation_field.getSFRotation()[3],4)
            else:
                t = self.robot.getTime()
                if round(t,0) - last_t >= 1:
                    last_t = round(t,0)
                    trajectory.append((cur_pos[0], cur_pos[2]))


            if doorAction != "front":
                if abs(cur_pos[0]) == 0.0 and abs(cur_pos[2]) <= 0.15 and last_x != 0:
                    print(cur_pos)
                    laps += 1
                    print("LAP:",laps)
            else:
                if abs(cur_pos[0]) == 0.0 and cur_pos[2] <= -0.7 and last_x != 0:
                    print(cur_pos)
                    laps += 1
                    print("LAP:",laps)
                
            last_x = cur_pos[0]
            
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
            F, FL, L, FR, R, B = [self.frontBrain.sensorVoltageToDistance(dsVal) for dsVal in dsValues]

            if corridor:
                # High Level State
                doorState = self.getDoorState((cur_pos[0], cur_pos[2], cur_ori))
                # Robot in door entrance and facing door
                if doorState != None:
                    print("DOOR:", statesF[doorState], "S:", doorState, "P:", (cur_pos[0], cur_pos[2], cur_ori), "A:", doorAction)
                    if doorAction == "right":
                        right = True
                        left  = False
                        front = False
                    elif doorAction == "left":
                        right = False
                        left  = True
                        front = False
                    elif doorAction == "front":
                        right = False
                        left  = False
                        front = True                
                    corridor = False
                
                # Corners
                elif F < 0.3 and B == 0.3:
                    # Right Corner
                    if FR == 0.3 and R == 0.3:
                        right = True
                        left = False
                        front = False
                        corridor  = False
                    # Left Corner
                    elif FL == 0.3 and L == 0.3:
                        right = False
                        left = True
                        front = False
                        corridor  = False
                            
            # Corridor
            elif FL < 0.3 and FR < 0.3 and R < FR and L < FL and F == 0.3 and (left or right or front):
                front    = False
                left     = False
                right    = False
                corridor = True
                
            # Current state of the robot
            state = self.frontBrain.sensorsToState(dsValues)        
            if left and not right and not front and not corridor:
                action = self.leftBrain.chooseAction(state)       
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
            
            if end or laps > 1000: # equivalent to 1000 real laps
                break

        trajectory.append((-5,-5))
        print("SAVING TRAJECTORY...")
        with open(name, "w+") as f:
            for p in trajectory:
                f.write(str(p) + "\n")
        print("TRAJECTORY SAVED!")

        print("Failed turns to", doorAction, ":", errors) 
        # Enter here exit cleanup code.
        exit(0)

    def getDoorState(self, state):
        x,z,ori = state
        # Create all possible combinations within margin
        all_states = []

        cur_ori = abs(ori)*180/math.pi
        # When checking the goal position consider square area of 6x6
        
        if str(state) in self.doors.keys():
            return str(state)
        # Cases where robot is facing up or down -> x represents corridor width
        if cur_ori < 20 or cur_ori > 160: 
            for i in range(-7,8):
                for j in range(-1,2):
                    for l in range(-20, 21):
                        all_states.append(str((round(x+i/100,2), round(z+j/100,2), round(abs(ori+(l*math.pi/180)),1))))
        
        # Cases where robot is facing sides -> z represents corridor width
        elif cur_ori > 70 and cur_ori < 110:
            for i in range(-1,2):
                for j in range(-7,8):
                    for l in range(-20, 21):
                        all_states.append(str((round(x+i/100,2), round(z+j/100,2), round(ori+(l*math.pi/180),1))))
        
        # Check if state exists
        for s in all_states:
            if s in self.doors.keys():
                return s
        return None

agentController()