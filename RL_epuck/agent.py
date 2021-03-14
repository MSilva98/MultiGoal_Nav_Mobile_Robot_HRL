import numpy as np
import itertools
import json
import math

class Agent():
    def __init__(self, alpha=0, beta=0, ro=0, epsilon=0.1, RWDTable=None, Qtable=None):
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

        if Qtable == None:
            self.QTable = {}
            for state in itertools.product(range(1,self.sensors_states+1), repeat=self.nr_sensors):
                self.QTable[state] = {}
                for a in self.actions:
                    self.QTable[state][a] = 0  # each state will have all actions
        else:
            with open(Qtable, "r") as jsonFile:
                self.QTable = json.load(jsonFile)

        if RWDTable == None:
            self.RwdTable = {}
            for state in itertools.product(range(1,self.sensors_states+1), repeat=self.nr_sensors):
                self.RwdTable[state] = 0
        else:
            with open(RWDTable, "r") as jsonFile:
                self.RwdTable = json.load(jsonFile)

    def chooseAction(self, state):
        action = ''
        if np.random.uniform(0,1) < self.epsilon:
            action = np.random.choice(self.actions)
        else:
            # https://thispointer.com/python-how-to-get-all-keys-with-maximum-value-in-a-dictionary/
            # Select the action with highest q_value (if more than one with the same max q_value then select a random one from those)
            maxAction = max(self.QTable[state].items(), key=lambda x: x[1])
            listofmaxActions = [k for k, v in self.QTable[state].items() if v == maxAction[1]]
            action = np.random.choice(listofmaxActions)
        return action

    def maxQ(self, state):
        highest_q = 0
        for a in self.actions:
            nxt_q = self.QTable[state][a]
            if nxt_q >= highest_q:
                highest_q = nxt_q
        return highest_q
    
    def discretizeSensor(self, sensor_val):
        if sensor_val < 7:
            return 1
        elif sensor_val >= 7 and sensor_val < 13:
            return 2
        elif sensor_val >= 13 and sensor_val < 20:
            return 3
        elif sensor_val >= 20:
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

    def sensorVoltageToDistance(self, voltage):
        # from webots documentation
        # TODO change this equation to count with offset
        # NOT JUST ADD 0.03 (negative sensors rip)
        return round((0.1594*pow(voltage,-0.8533))-0.02916,4) # in meters rounded to 3 decimal cases

    def reward(self, pos, ori):
        # TODO - Create my reward
        # Reward from Diogo's dissertation
        return -500*(pow(pos,2)+0.0159*pow(ori,2))+2

    def getPosOriFromSensors(self, sensor_values):
        # matlab sensors position       [0 pi/4 pi/2 pi -pi/2 -pi/4]
        # indexes                       [1  2    3    4   5     6]
        # webots corresponding indexes  [0  1    2    5   4     3]

        # REAL VALUES FROM WEBOTS
        dist_IR_sensor = []
        for s in sensor_values:
            d = self.sensorVoltageToDistance(s)
            if d > 0.3:
                d = 0.3
            dist_IR_sensor.append(d)   

        # Values from matlab with x = 0.04 and fi = pi/2
        # Equivalent to webots x = -0.11 and rot = 0
        # dist_IR_sensor = [0.3, 0.0566, 0.04, 0.3, 0.26, 0.3]
        
        fs = dist_IR_sensor[0]  # Front sensor - d1
        bs = dist_IR_sensor[5]  # Back sensor  - d4
        ls = dist_IR_sensor[2]  # Left sensor  - d3
        rs = dist_IR_sensor[4]  # Right sensor - d5
        
        # Position and Orientation in corridor
        if fs != 0.3 and bs != 0.3:
            pos = 0.15*(bs-fs)/(bs+fs)
            ori = math.pi/2-math.acos((0.15-pos)/fs)
        elif ls != 0.3 and rs != 0.3:
            pos = 0.15*(ls-rs)/(ls+rs)
            ori = math.acos((0.15-pos)/rs)
        else:
            print("ERROR: Sensor Distances out of range")
 
        # Absolute values
        pos = abs(pos) 
        ori = abs(ori)
        sol = [(pos, ori), (pos, -ori), (-pos, ori), (-pos, -ori)] # All 4 possible solutions
        
        i_sol = [0,1,2,3]
        P_R = np.matrix([
            [0, -dist_IR_sensor[1]*math.sqrt(2)/2, -dist_IR_sensor[2], 0, dist_IR_sensor[4], dist_IR_sensor[3]*math.sqrt(2)/2],
            [dist_IR_sensor[0], dist_IR_sensor[1]*math.sqrt(2)/2, 0, -dist_IR_sensor[5], 0, dist_IR_sensor[3]*math.sqrt(2)/2],
            [1, 1, 1, 1, 1, 1]
        ])

        for k in range(0,4):
            # Coordinate transformation
            T_R2L = np.matrix([
                [math.cos(sol[k][1]), -math.sin(sol[k][1]), sol[k][0]], 
                [math.sin(sol[k][1]), math.cos(sol[k][1]), 0], 
                [0, 0, 1]
            ])

            # Reference frame new coordinates
            P_L = []
            for m in range(len(sensor_values)):
                if dist_IR_sensor[m] < 0.3:
                    P_L = T_R2L*P_R[:,m]
            
            # Get matrix first column and convert to list
            tmp = np.array(P_L[0,:]).reshape(-1,).tolist()
            # Absolute value of each element
            P_L_xx = [abs(x) for x in tmp]
            for m in range(len(P_L_xx)):
                if abs(P_L_xx[m]-0.15) > 1e-8:
                    i_sol.remove(k)
                    break

        return sol[i_sol[0]]
        
    def sensorsToState(self, sensor_values):
        return tuple([self.discretizeSensor(self.sensorVoltageToDistance(x)) for x in sensor_values])

    # sensor_values organized from sharp0 to sharp5
    # [front, front left, left, front right, right, rear]
    def fillRwdTable(self, sensor_values):
        state = self.sensorsToState(sensor_values)
        pos, ori = self.getPosOriFromSensors(sensor_values)
        rwd = self.reward(pos, ori)
        self.RwdTable[state] = rwd
        
    # sensor_values organized from sharp0 to sharp5
    # update QTable value for each action
    # R-Learning implementation from Sutton&Barto 1998
    def updateQTable(self, cur_state, sensor_values, action):
        next_state = self.sensorsToState(sensor_values)
        rwd = self.RwdTable[next_state]
        cur_q = self.QTable[cur_state][action]
        new_q = cur_q+self.alpha(rwd-self.ro+self.maxQ(next_state)-cur_q)
        self.QTable[cur_state][action] = new_q

        if new_q == self.maxQ(cur_state):
            self.ro = self.ro+self.beta(rwd-self.ro+self.maxQ(next_state)-self.maxQ(cur_state))
    
    def updateQTable_hLearning(self, cur_state, sensor_values, action):
        pass

    def saveQTable(self):
        with open("QTable.txt", "w+") as jsonFile:
            json.dump(self.QTable, jsonFile)

    def saveRwdTable(self):
        with open("RwdTable.txt", "w+") as jsonFile:
            json.dump(self.RwdTable, jsonFile)

    
a = Agent()

print(a.sensorVoltageToDistance(150))