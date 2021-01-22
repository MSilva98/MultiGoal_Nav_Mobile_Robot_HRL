"""data_collect_khepera controller."""
# python script to collect data from Sharp and infrared distance sensors

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np
import time
import sys

# print without scientific notation
np.set_printoptions(suppress=True)

class data_collect():
    def __init__(self):
        # create supervisor instance (Set of functions available for each robot node)
        self.supervisor = Supervisor()
        # # create the Robot instance.
        # self.supervisor = Robot()

        # node to use supervisor functions
        self.robot_node = self.supervisor.getFromDef("khepera4")
        self.rotation_node = self.robot_node.getField("rotation")

        # get the time step of the current world.
        # timestep = int(robot.getBasicTimeStep())
        self.timestep = 64

        self.MAX_SPEED = 47.6

        # # initialize devices
        # self.ds = []
        # self.dsNames = [
        #     'sharps0',  # front
        #     'sharps1',  # front left
        #     'sharps2',  # left
        #     'sharps3',  # front right
        #     'sharps4',  # right
        #     'sharps5'   # rear
        # ]
        # # enable sharp distance sensors
        # for i in range(len(self.dsNames)):
        #     self.ds.append(self.supervisor.getDistanceSensor(self.dsNames[i]))
        #     self.ds[i].enable(self.timestep)
        # # All 6 sensors are equal so their lookup Table is equal
        # self.sharpLookupTable = self.getLookupTable(self.us[0]) 
        
        self.us = []
        self.usNames = [
            'left ultrasonic sensor',
            'front left ultrasonic sensor',
            'front ultrasonic sensor',
            'right ultrasonic sensor',
            'front right ultrasonic sensor'
        ]
        # enable ultra-sonic sensors
        for i in range(len(self.usNames)):
            self.us.append(self.supervisor.getDistanceSensor(self.usNames[i]))
            self.us[i].enable(self.timestep)
        
        # All 5 sensors are equal so their lookup Table is equal
        self.usLookupTable = self.getLookupTable(self.us[0]) 
        
        # self.ir = []
        # self.irNames = [
        #     'ps0',	# front front right
        #     'ps1',	# front right
        #     'ps2',	# right
        #     'ps3',	# rear right
        #     'ps4',	# rear left
        #     'ps5',	# left
        #     'ps6',	# front left
        #     'ps7'	# front front left
        # ]
        self.ir = []
        self.irNames = [
            'rear left infrared sensor',
            'rear infrared sensor',
            'rear right infrared sensor',
            'left infrared sensor',
            'right infrared sensor',
            'front left infrared sensor',
            'front right infrared sensor',
            'front infrared sensor',
            # NOT USED HERE
            # 'ground left infrared sensor',
            # 'ground front left infrared sensor',
            # 'ground front right infrared sensor',
            # 'ground right infrared sensor',
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

        self.data = open("data.csv", "a+")
        # self.data.write("sharps0, sharps1, sharps2, sharps3, sharps4, sharps5, ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7, x, y, z, theta\n")
        # self.data.write("left-us, front-left-us, front-us, front-right-us, right-us, rear-left-ir, rear-ir, rear-right-ir, left-ir, right-ir, front-left-ir, front-right-ir, front-ir, x, z, theta\n")
        self.run()

    def run(self):
        # Main loop:
        # - perform simulation steps until Webots is stopping the controller
        # Only write to file when count >= 10
        count = 0
        lines = 0

        while self.supervisor.step(self.timestep) != -1:
            psValues = []
            if count >= 20:
                count = 0
                print("save data ", lines)
                s = ""
                # read sharp sensors outputs
                for i in range(len(self.us)):
                    s += str(round(self.us[i].getValue(), 2)) + ", "

                # read IR sensors outputs
                for i in range(len(self.ir)):
                    v = self.ir[i].getValue()
                    psValues.append(v)
                    s += str(round(v,2)) + ", "

                pos = [round(i,2) for i in self.robot_node.getPosition()]
                rotation = round(self.rotation_node.getSFRotation()[3], 2)  # x,y,z ignored, just rotation
                s += str(pos[0]) + ", " + str(pos[2]) + ", " + str(rotation) + "\n"     # x, z only, y axis is height, not considered
                self.data.write(s)
                lines += 1
            else:
                # read IR sensors outputs
                for i in range(len(self.ir)):
                    psValues.append(self.ir[i].getValue())                
            
            speed_offset = 0.2 * (self.MAX_SPEED - 0.03 * self.ir[7].getValue())
            # speed_delta = 0.05 * ((self.ir[3].getValue() + self.ir[5].getValue())/2) - 0.05 * ((self.ir[4].getValue() + self.ir[6].getValue())/2)
            speed_delta = 0.031 * self.ir[5].getValue() + 0.02 * self.ir[3].getValue() - 0.029 * self.ir[6].getValue() - 0.02 * self.ir[4].getValue()

            self.leftMotor.setVelocity((speed_offset + speed_delta))
            self.rightMotor.setVelocity((speed_offset - speed_delta))
            
            count += 1

        self.data.close()
        # Enter here exit cleanup code.
        sys.exit(0)


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

