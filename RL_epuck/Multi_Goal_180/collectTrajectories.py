"""run_Agent controller."""
# python script to control the robot with a QTable after training

# You may need to import some classes of the controller module. Ex:
from controller import Supervisor
import numpy as np
from lowLevel import Agent as lowLvlAgent
from highLevel import Agent as highLvlAgent
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
        # Symetry can be used instead (which is better computationally?)
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
        init_pos = 0

        # Select maze
        maze = 2

        # Robot Start Positions
        if maze == 1:
            # MAZE 1
            self.highLevel = highLvlAgent(epsilon=0, statesFile="./mazes/maze1.txt", Qtable="QTable_HighLevel_maze1.txt")
            init_pos1 = [0.85,0,0.85]
            init_ori1 = [0,1,0,1.57]
            init_pos2 = [-0.85,0,0.65]
            init_ori2 = [0,1,0,0]
            init_pos3 = [-0.65,0,-0.85]
            init_ori3 = [0,1,0,-1.57]
            init_pos4 = [-0.15,0,-0.55]
            init_ori4 = [0,1,0,-1.57]
            name = "maze1_trajectory.txt"
        else:
            # MAZE 2
            self.highLevel = highLvlAgent(epsilon=0, statesFile="./mazes/maze2.txt", Qtable="QTable_HighLevel_maze2.txt")
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
            name = "maze2_trajectory.txt"

        # Set robot intial position
        self.translation_field.setSFVec3f(init_pos1)
        self.rotation_field.setSFRotation(init_ori1)
        self.robot_node.resetPhysics()

        # Trajectory collection
        trajectory = []
        last_t = 0
        
        startGoal = 'home1'
        finishGoal = 'gym1' 

        while self.robot.step(self.timestep) != -1:    
            # Current position of robot each timestep
            cur_pos = [round(p,2) for p in self.translation_field.getSFVec3f()]
            cur_ori = round(self.rotation_field.getSFRotation()[3],4)
            
            t = self.robot.getTime()
            if round(t,0) - last_t >= 1:
                last_t = round(t,0)
                trajectory.append((cur_pos[0], cur_pos[2]))
            
            # Each timestep check if robot reached goal
            if self.highLevel.reachedGoal((cur_pos[0], cur_pos[2], cur_ori), finishGoal):
                init_pos += 1
                # RESET ROBOT
                if init_pos < 10:
                    trajectory.append((-5, -5))
                    self.translation_field.setSFVec3f(init_pos1)
                    self.rotation_field.setSFRotation(init_ori1)
                elif init_pos >= 10 and init_pos < 20:
                    trajectory.append((-5, -5))
                    self.translation_field.setSFVec3f(init_pos2)
                    self.rotation_field.setSFRotation(init_ori2)
                elif init_pos >= 20 and init_pos < 30: 
                    trajectory.append((-5, -5))
                    self.translation_field.setSFVec3f(init_pos3)
                    self.rotation_field.setSFRotation(init_ori3)
                elif init_pos >= 30 and init_pos < 40:
                    trajectory.append((-5, -5))
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
            F, FL, L, FR, R, B = [self.frontBrain.sensorVoltageToDistance(dsVal) for dsVal in dsValues]

            if corridor:
                # High Level State
                highState = self.highLevel.getState((cur_pos[0], cur_pos[2], cur_ori))
                # Robot in door entrance and facing door
                if highState != '':
                    highAction = self.highLevel.chooseAction(highState)
                    print("DOOR:", self.highLevel.getDoorName(highState), "S:", highState, "P:", (cur_pos[0], cur_pos[2], cur_ori), "A:", highAction)
                    if highAction == "right":
                        right = True
                        left  = False
                        front = False
                        corridor = False
                    elif highAction == "left":
                        right = False
                        left  = True
                        front = False
                        corridor = False
                    elif highAction == "front":
                        right = False
                        left  = False
                        front = True
                        corridor = False
                    elif highAction == "back":
                        self.turn_on_axis(180,1)             
                
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
            if end:
                break

        trajectory.append((-5,-5))
        print("SAVING TRAJECTORY...")
        with open(name, "w+") as f:
            for p in trajectory:
                f.write(str(p) + "\n")
        print("TRAJECTORY SAVED!")
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

agentController()