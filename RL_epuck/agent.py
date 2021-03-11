import numpy as np
import itertools
import json

class Agent():
    def __init__(self, lr=0.3, gamma=0.99, epsilon=0.1):
        self.lr = lr
        self.gamma = gamma
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

        self.QTable = {}
        for state in itertools.product(range(1,self.sensors_states+1), repeat=self.nr_sensors):
            self.QTable[state] = {}
            for a in self.actions:
                self.QTable[state][a] = 0  # each state will have all actions

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

    def volageToDistance(self, voltage):
        # from webots documentation
        return ((0.1594*pow(voltage,-0.8533))-0.02916)*100 # *100 -> m to cm

    def reward(self, pos, ori):
        # TODO
        return 0

    def getPositionFromSensors(self, sensor_values):
        # TODO
        return 0

    def getOrientationFromSensors(self, sensor_values):
        # TODO
        return 0

    # sensor_values organized from sharp0 to sharp5
    # [front, front left, left, front right, right, rear]
    def fillQTable(self, sensor_values):
        state = tuple([self.discretizeSensor(self.volageToDistance(x)) for x in sensor_values])
       
        pos = self.getPositionFromSensors(sensor_values)
        ori = self.getOrientationFromSensors(sensor_values)
        rwd = self.reward(pos, ori)
        
        for a in self.actions:
            self.QTable[state][a] = rwd

    # sensor_values organized from sharp0 to sharp5
    def updateQTable(self, sensor_values):
        state = 1
        for s in sensor_values:
            state *= self.discretizeSensor(self.volageToDistance(s))
        # TODO
    
    def saveQTable(self):
        f = open("QTable.txt", "w+")
        for s in self.QTable:
            f.write("{" + str(s) + ": " + json.dumps(self.QTable[s]) + "}\n")
        f.close()