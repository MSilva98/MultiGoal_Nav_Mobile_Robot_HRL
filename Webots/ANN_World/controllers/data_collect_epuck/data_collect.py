"""data_collect controller."""
# python script to collect data from Sharp and infrared distance sensors

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time

# print without scientific notation
np.set_printoptions(suppress=True)

class data_collect():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()
        # # create the Robot instance.
        # self.robot = Robot()

        # node to use supervisor functions
        self.robot_node = self.supervisor.getFromDef("epuck")
        self.rotation_node = self.robot_node.getField("rotation")

        # get the time step of the current world.
        # timestep = int(robot.getBasicTimeStep())
        self.timestep = 64

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
            self.ds.append(self.supervisor.getDistanceSensor(self.dsNames[i]))
            self.ds[i].enable(self.timestep)
        
        # All 6 sensors are equal so their lookup Table is equal
        self.sharpLookupTable = self.getLookupTable(self.ds[0]) 
        
        self.ir = []
        self.irNames = [
            'ps0',	# front front right
            'ps1',	# front right
            'ps2',	# right
            'ps3',	# rear right
            'ps4',	# rear left
            'ps5',	# left
            'ps6',	# front left
            'ps7'	# front front left
        ]
        # enable infrared distance sensors
        for i in range(len(self.irNames)):
            self.ir.append(self.supervisor.getDistanceSensor(self.irNames[i]))
            self.ir[i].enable(self.timestep)

        # All 8 sensors are equal so their lookup Table is equal
        self.irLookupTable = self.getLookupTable(self.ir[0])

        self.leftMotor = self.supervisor.getMotor('left wheel motor')
        self.rightMotor = self.supervisor.getMotor('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        self.data = open("data.csv", "w+")
        self.data.write("sharps0, sharps1, sharps2, sharps3, sharps4, sharps5, ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7, x, y, z, theta\n")
        self.run()

    def run(self):
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        # Only write to file when count >= 10
        count = 0   
        random = 0
        val = (0.52, 0.48)

        while self.supervisor.step(self.timestep) != -1:
            psValues = []
            if count >= 20:
                count = 0
                print("save data")
                s = ""
                # read sharp sensors outputs
                for i in range(len(self.ds)):
                    s += str(round(self.ds[i].getValue(), 4)) + ", "

                # read IR sensors outputs
                for i in range(len(self.ir)):
                    v = self.ir[i].getValue()
                    psValues.append(v)
                    s += str(round(v,4)) + ", "

                pos = [round(i,4) for i in self.robot_node.getPosition()]
                rotation = round(self.rotation_node.getSFRotation()[3], 4)  # x,y,z ignored, just rotation
                s += str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[2]) + ", " + str(rotation) + "\n"
                self.data.write(s)
            else:
                # read IR sensors outputs
                for i in range(len(self.ir)):
                    psValues.append(self.ir[i].getValue())                
            
            # detect obstacles
            right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0 
            left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

            if random >= 200:
                print(val)
                val = (val[1], val[0])
                random = 0
                print(val)

            # initialize motor speeds at 50% of MAX_SPEED.
            leftSpeed  = val[0] * self.MAX_SPEED
            rightSpeed = val[1] * self.MAX_SPEED
            # modify speeds according to obstacles
            if left_obstacle:
                # turn right
                leftSpeed  += 0.5 * self.MAX_SPEED
                rightSpeed -= 0.5 * self.MAX_SPEED
            elif right_obstacle:
                # turn left
                leftSpeed  -= 0.5 * self.MAX_SPEED
                rightSpeed += 0.5 * self.MAX_SPEED
            
            # write actuators inputs
            self.leftMotor.setVelocity(leftSpeed)
            self.rightMotor.setVelocity(rightSpeed)
            count += 1
            random += 1

        self.data.close()
            # Enter here exit cleanup code.


    def getLookupTable(self, distSensor):
        l = distSensor.getLookupTable()
        # convert list to list of lists with (distance, sensor value, noise)  
        return np.array([l[j:j+3] for j in range(0, len(l), 3)])

    # Matrix of 9 that can be used to calculate the local coordinates given the world ones
    def getOrientation(self):
        m = self.robot_node.getOrientation()
        return np.array([m[j:j+3] for j in range(0, len(m), 3)])

    # Not very good since lookup table only has a few values
    def getDistance(self, sensorVal, lkT):
        for i in lkT:
            if sensorVal == i[1]: # middle column has the sensor values
                return i[0]
        return 0


data_collect()
