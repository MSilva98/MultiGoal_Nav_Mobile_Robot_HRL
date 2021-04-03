import numpy as np
import itertools
import math
import json

class Agent():
    def __init__(self, alpha=0.3, beta=0.3, ro=0, epsilon=0.1, RWDTable=None, Qtable=None):
        self.alpha = alpha
        self.beta = beta
        self.ro = ro
        self.epsilon = epsilon
        # Actions the agent can perform
        # Front, Light Left, Light Right, Mid Left, Mid Right, Left, Right, Hard Left, Hard Right
        self.actions = ["F", "LL", "LR", "ML", "MR", "L", "R", "HL", "HR"]
        self.sensors_states = 4 # 4 discretization states -> 1 - Very Close, 2 - Close, 3 - Good, 4 - Far
        self.nr_sensors = 6     # 6 sharp sensors -> sharp0, sharp1, sharp2, sharp3, sharp4, sharp5 -> front, front left, left, front right, right, rear
        # States will go from (1,1,1,1,1,1) to (4,4,4,4,4,4) = (front, front left, left, front right, right, rear)
        # state 1    (1,1,1,1,1,1)
        # state 2    (1,1,1,1,1,2)
        # state 3    (1,1,1,1,1,3)
        # state 4    (1,1,1,1,1,4)
        # state 5    (1,1,1,1,2,1)
        # ...
        # state 4095 (4,4,4,4,4,3)
        # state 4096 (4,4,4,4,4,4)
        self.randomAct = 0  # control if random action was choosen (1 - random, 0 - non-random)

        if Qtable == None:
            self.QTable = {}
            for state in itertools.product(range(1,self.sensors_states+1), repeat=self.nr_sensors):
                self.QTable[str(state)] = {}
                for a in self.actions:
                    self.QTable[str(state)][a] = 0  # each state will have all actions
        else:
            with open(Qtable, "r") as jsonFile:
                self.QTable = json.load(jsonFile)

        if RWDTable == None:
            self.RwdTable = {}
            for state in itertools.product(range(1,self.sensors_states+1), repeat=self.nr_sensors):
                self.RwdTable[str(state)] = 0
        else:
            with open(RWDTable, "r") as jsonFile:
                self.RwdTable = json.load(jsonFile)

    def chooseAction(self, state):
        action = ''
        if np.random.uniform(0,1) < self.epsilon:
            action = np.random.choice(self.actions)
            self.randomAct = 1
        else:
            # https://thispointer.com/python-how-to-get-all-keys-with-maximum-value-in-a-dictionary/
            # Select the action with highest q_value (if more than one with the same max q_value then select a random one from those)
            maxAction = max(self.QTable[state].items(), key=lambda x: x[1])
            listofmaxActions = [k for k, v in self.QTable[state].items() if v == maxAction[1]]
            action = np.random.choice(listofmaxActions)
            self.randomAct = 0
        return action

    def maxQ(self, state):
        highest_q = 0
        for a in self.actions:
            nxt_q = self.QTable[state][a]
            if nxt_q >= highest_q:
                highest_q = nxt_q
        return highest_q
    
    def discretizeSensor(self, sensor_val):
        if sensor_val > 0 and sensor_val < 0.07:
            return 1
        elif sensor_val >= 0.07 and sensor_val < 0.13:
            return 2
        elif sensor_val >= 0.13 and sensor_val < 0.20:
            return 3
        elif sensor_val >= 0.20:
            return 4
        else:   # bad sensor value
            return -1
    
    def actionToSpeed(self, action):
        if action == "F":
            return (2,2)
        elif action == "LL":
            return (1.5,2)
        elif action == "LR":
            return (2,1.5)
        elif action == "ML":
            return (1,2)
        elif action == "MR":
            return (2,1)
        elif action == "L":
            return (0,2)
        elif action == "R":
            return (2,0)
        elif action == "HL":
            return (-2,2)
        elif action == "HR":
            return (2,-2)
        else:   # bad action
            return -1

    def sensorVoltageToDistance_Webots(self, voltage):
        # from webots documentation
        # Add 0.03 to compensate the sensor offset to the robot center
        d = ((0.1594*pow(voltage,-0.8533))-0.02916)        
        return d if d <= 0.3 else 0.3

    def sensorVoltageToDistance(self, voltage, b=False):
        # from IR_convert_to_cm.py
        # Each sensor has a 0.03m offset from robot center 
        # Divide by 100 to convert to meters
        # To improve conversion from voltage to distance three equations are considered to different voltages
        if voltage <= 0.48:
            d = (15.065187821049603/(voltage+0.04822905725919005)-1.4)/100
        elif voltage > 0.48 and voltage <= 0.53:
            d = (14.483674005107527/(voltage+0.02681880954262667)-1.2)/100
        else:
            d = (13.045778715415159/(voltage-0.028295530064741125)-0.7)/100
        
        return d if d <= 0.3 else 0.3

    # Pos -> -0.11 to 0.11
    # Ori -> -1.57 to 1.57
    def reward(self, pos, ori):
        # Reward from Diogo's dissertation
        # return -500*(pow(pos,2)+0.0159*pow(ori,2))+2
        
        # # newRWD
        # if (pos >= 0 and ori >= 0) or (pos <= 0 and ori <= 0):  # Cases when robot is moving to the center
        #     return -500*(pow(pos,2)+0.0159*pow(ori,2))+2
        # else:
        #     return -500*(pow(pos,2)+0.0159*pow(ori,2))-2        # Cases when it is walking away
        
        # # V2
        # r = -150*(pow(pos,2)+0.0159*pow(ori,2))
        # if (pos >= 0 and ori >= 0) or (pos <= 0 and ori <= 0):  # Cases when robot is moving to the center
        #     return r+2
        # else: # Cases when it is walking away
        #     return r-8        

        # V3
        if (pos >= 0 and ori >= 0) or (pos <= 0 and ori <= 0):  # Cases when robot is moving to the center
            return -200*(2*pow(pos,2)+0.01*pow(ori,2))+4
        else: # Cases when it is walking away
            return -200*(pow(pos,2)+0.02*pow(ori,2))-6     

        # # V4
        # if (pos >= 0 and ori >= 0) or (pos <= 0 and ori <= 0):  # Cases when robot is moving to the center
        #     return -300*(4*pow(pos,2)+0.02*pow(ori,2))+4
        # else: # Cases when it is walking away
        #     return -300*(4*pow(pos,2)+0.05*pow(ori,2))+4    # Penalizes much harder rotations

        
    def sensorsToState(self, sensor_values):
        return str(tuple([self.discretizeSensor(round(self.sensorVoltageToDistance(v),2)) for v in sensor_values]))

    # sensor_values organized from sharp0 to sharp5
    # [front, front left, left, front right, right, rear]
    def fillRwdTable(self, sensor_values, pos, ori, prev_state):
        dists = [round(self.sensorVoltageToDistance(v),2) for v in sensor_values]
        state = self.sensorsToState(sensor_values)
        rwd = round(self.reward(pos[0], ori),4)
        print("POS:", pos[0], pos[2], " ORI:", round(ori*180/math.pi,1))
        if prev_state != state:
            with open("rwd_log.txt", "a+") as f:
                f.write("Prev State: " + prev_state + "\nCur State: " + state + "\nDists:" + str(dists) + "\nPos:" + str(pos[0]) + " ORI:" + str(ori) + " " + str(ori*180/math.pi) + "\nRWD:" + str(rwd) + "\n")
            prev_state = state
        self.RwdTable[state] = rwd
        return state
        
    # sensor_values organized from sharp0 to sharp5
    # update QTable value for each action
    # R-Learning implementation from
    # Average Reward Reinforcement Learning: Foundations, Algorithms, and Empirical Results SRIDHAR MAHADEVAN 
    # alpha and beta changed: alpha - adjust relative action values R(s,a), beta - ajust average reward
    # Low alpha is better than high alpha (default 0.05)
    # High beta is better than low beta   (default 0.5)
    def updateQTable(self, cur_state, next_state, action):
        rwd = self.RwdTable[next_state]
        cur_q = self.QTable[cur_state][action]
        new_q = cur_q*(1-self.alpha)+self.alpha*(rwd-self.ro+self.maxQ(next_state))
        self.QTable[cur_state][action] = new_q
        print(" STATE:", cur_state, " Action:", action, " RWD:", rwd)
        if new_q == self.maxQ(cur_state) and not self.randomAct:
            self.ro = self.ro*(1-self.beta)+self.beta*(rwd+self.maxQ(next_state)-self.maxQ(cur_state))
    
    # R-Learning implementation from Sutton&Barto 1998
    def updateQTable_SuttonBarto(self, cur_state, next_state, action):
        rwd = self.RwdTable[next_state]
        cur_q = self.QTable[cur_state][action]
        new_q = cur_q+self.alpha*(rwd-self.ro+self.maxQ(next_state)-cur_q)
        self.QTable[cur_state][action] = new_q
        print(" STATE:", cur_state, " Action:", action, " RWD:", rwd)
        if new_q == self.maxQ(cur_state) and not self.randomAct:
            self.ro = self.ro+self.beta*(rwd-self.ro+self.maxQ(next_state)-self.maxQ(cur_state))
    
    def saveQTable(self):
        with open("QTable.txt", "w+") as jsonFile:
            json.dump(self.QTable, jsonFile)

    def saveRwdTable(self):
        with open("RwdTable.txt", "w+") as jsonFile:
            json.dump(self.RwdTable, jsonFile)


    ##########################
    ######## NOT USED ########
    ##########################
    # Given the fact that this relies in geometry (arccos) any sensor noise will affect the final solution
    # and, in some cases, it won't find one
    # Example: with the robot with x=0.00 and theta=0, Right Sensor might give values like 0.152
    # corresponding to arccos(0.15/0.152) which is 0.16 rad = 9.17 degrees
    def getPosOriFromSensors(self, sensor_values, prev_state, logEnabled=False):
        # matlab sensors position       [0 pi/4 pi/2 pi -pi/2 -pi/4]
        # indexes                       [1  2    3    4   5     6]
        # webots corresponding indexes  [0  1    2    5   4     3]

        if logEnabled:
            log = open("log.txt", "a+")
            # log = open("toDelete.txt", "w+")
            log.write("====================================================\n"+
                    "==================== NEW READ ======================\n"+
                    "====================================================\n")
            log.write("EPUCK Previous State: " + str(prev_state) + "\n")
        
        # Convert sensor outputs from webots to distances
        dist_IR_sensor = []
        for s in sensor_values:
            d = round(self.sensorVoltageToDistance(s),4)
            # d = round(self.sensorVoltageToDistance_Webots(s),4)

            dist_IR_sensor.append(d)   
    
        if logEnabled:
            log.write("Webots: " + str(dist_IR_sensor) + "\n")

        fs = dist_IR_sensor[0]  # Front sensor - d1
        bs = dist_IR_sensor[5]  # Back sensor  - d4
        ls = dist_IR_sensor[2]  # Left sensor  - d3
        rs = dist_IR_sensor[4]  # Right sensor - d5

        # Position and Orientation relative to wall
        if ls != 0.3 and rs != 0.3:
            pos = round(0.15*(ls-rs)/(ls+rs),3)
            tmp = (0.15-pos)/rs
            if tmp > 1:
                tmp = 1
            ori = round(math.acos(tmp),4)
        elif fs != 0.3 and bs != 0.3:
            pos = round(0.15*(fs-bs)/(fs+bs),3)
            tmp = (0.15-pos)/bs
            if tmp > 1:
                tmp = 1
            ori = round(math.pi/2-math.acos(tmp),4)
        else:
            print("ERROR: Sensor Distances out of range")

        # Absolute values
        pos = abs(pos) 
        ori = abs(ori)
        if logEnabled:
            log.write("\nPOS: " + str(pos) + "\nORI: " + str(ori) + "\n")
        
        # All possible solutions in 360 degrees
        if ori not in [0, math.pi/2, math.pi]: 
            if pos != 0:
                sols = [(pos, ori), (pos, -ori), (pos, ori-math.pi), (pos, math.pi-ori), (-pos, ori), (-pos, -ori), (-pos, ori-math.pi), (-pos, math.pi-ori)] 
                possible_sols = [0, 1, 2, 3, 4, 5, 6, 7]
            else:
                sols = [(pos, ori), (pos, -ori), (pos, ori-math.pi), (pos, math.pi-ori)] 
                possible_sols = [0, 1, 2, 3]        
        else:   # When in 0, 90, 180, -90 solutions trimmed to only 4
            if pos != 0:
                sols = [(pos, ori), (pos, ori-math.pi), (-pos, ori), (-pos, ori-math.pi)] 
                possible_sols = [0, 1, 2, 3]
            else:
                sols = [(pos, ori), (pos, ori-math.pi)] 
                possible_sols = [0, 1]

        # Intersection points in the robot frame in order:
        # Line 1 = x
        # Line 2 = y
        # Line 3 = z (uniform -> 2d sensor measure)
        # Order: Front, Front Left, Left, Front Right, Right, Back
        P_R = np.matrix([
            [0, -dist_IR_sensor[1]*math.sqrt(2)/2, -dist_IR_sensor[2], dist_IR_sensor[3]*math.sqrt(2)/2, dist_IR_sensor[4], 0],
            [dist_IR_sensor[0], dist_IR_sensor[1]*math.sqrt(2)/2, 0, dist_IR_sensor[3]*math.sqrt(2)/2, 0, -dist_IR_sensor[5]],
            [1, 1, 1, 1, 1, 1]
        ])
        if logEnabled:
            log.write("\nP_R\n" + str(P_R) + "\n")
        
        for k in range(0,len(sols)):
            # Coordinate transformation from the robot frame to a frame localized in the center 
            # of the corridor with origin in the same Y as the robot frame
            T_R2L = np.matrix([
                [math.cos(sols[k][1]), -math.sin(sols[k][1]), sols[k][0]], 
                [math.sin(sols[k][1]), math.cos(sols[k][1]), 0], 
                [0, 0, 1]
            ])

            if logEnabled:
                log.write("\nITERATION: " + str(k))
                log.write("\nT_R2L\n" + str(T_R2L) + "\n")

            # Reference frame new coordinates
            PL = []
            for m in range(len(sensor_values)):
                if dist_IR_sensor[m] < 0.3:
                    if len(PL) == 0:
                        PL = T_R2L*P_R[:,m]
                    else:
                        PL = np.hstack((PL, T_R2L*P_R[:,m]))
            
            if logEnabled:
                log.write("\nPL\n" + str(PL) + "\n")

            # Get matrix first column and convert to list
            tmp = np.array(PL[0,:]).reshape(-1,).tolist()
            # Absolute value of each element
            PL_xx = [round(abs(x),3) for x in tmp]
            
            if logEnabled:
                log.write("\nPL_xx\n" + str(PL_xx) + "\n")
                log.write("\nSolution being tested\n" + str(sols[k]) + "\n")
            
            for x in PL_xx:
                if round(abs(x-0.15),2) > 0.015:     # if not all values in PL_xx are equal to 0.15 not a SOLUTION
                    possible_sols.remove(k)
                    break

        # Obtain final solution
        # When robot was not centered and is not centered 
        sols = [sols[s] for s in possible_sols]
        if prev_state[0] != 0 and round(sols[0][0],2) != 0:
            dists = [abs(prev_state[0]-round(s[0],3)) for s in sols]
            min_dist_idx = dists.index(min(dists))
            sol = sols[min_dist_idx]
        # Case when robot centered use angle
        else:
            if prev_state[1] >= 170:
                angles = [abs(prev_state[1]-abs(round(s[1]*180/math.pi,1))) for s in sols]
            else:
                angles = [abs(prev_state[1]-round(s[1]*180/math.pi,1)) for s in sols]
            min_angle_idx = angles.index(min(angles))
            sol = sols[min_angle_idx]

        final_sol = (round(sol[0],3), round(sol[1]*180/math.pi,1))
        
        if logEnabled:
            log.write("\nAll Solutions: " + str([(round(s[0],3), round(s[1]*180/math.pi,1)) for s in sols]))
            log.write("\nSolution: " + str(final_sol))
            log.write("\n===================================================="+
                  "\n==================== END READ ======================"+
                  "\n====================================================\n\n")
        
        return final_sol
