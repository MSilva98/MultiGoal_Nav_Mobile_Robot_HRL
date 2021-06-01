from os import stat
import numpy as np
import itertools
import math
import json

class Agent():
    def __init__(self, alpha=0.3, gamma=0.99, episodes=500, epsilon=0.1, statesFile=None, Qtable=None, memoryDoors=None):
        self.alpha = alpha
        self.gamma = gamma
        self.episodes = episodes
        self.epsilon = epsilon

        # States -> Will be each door entrance - DEPEND ON EACH MAZE
        # Each maze has a statesFile with dictionary of type -> {"(x, z, ori)": "doorType",}
        # Last position in dictionary defines goal -> "(x, z)": "goal"
        # NOTE: The space between x, z and ori in (x, z, ori) is MANDATORY as well as in (x, z)
        if statesFile == None:
            print("MUST DEFINE STATES BEFORE")
            exit()
        else:
            with open(statesFile, "r") as jsonFile:
                self.statesF = json.load(jsonFile)

            with open(statesFile[:-4] + "_names.txt", "r") as jsonFile:
                self.statesNames = json.load(jsonFile)

        # Actions the agent can perform
        # Depend on type of door
        # FL Door: Front or Left
        # FR Door: Front or Right
        # LR Door: Left or Right
        # FLR Door: Front or Left or Right (not implemented in the Low Level)

        self.states = []
        if Qtable == None:
            self.QTable = {}
            for state in self.statesF:
                self.QTable[state] = {}
                if self.statesF[state] == "fr":
                    actions = ["front", "right"]
                elif self.statesF[state] == "fl":
                    actions = ["front", "left"]
                elif self.statesF[state] == "lr":
                    actions = ["left", "right"]
                elif self.statesF[state] == "flr":
                    actions = ["front", "left", "right"]
                elif self.statesF[state] == "goal":
                    self.goal = state
                for a in actions:
                    self.QTable[state][a] = 0  # each state will have all actions
        else:
            with open(Qtable, "r") as jsonFile:
                self.QTable = json.load(jsonFile)
            for state in self.statesF:
                if self.statesF[state] == "goal":
                    self.goal = state

        # Memory from blocked doors
        # When the webots controller terminates all the variables are cleaned
        # Robot won't know a path was blocked in before run... this file saves all doors that lead to blocked paths
        if memoryDoors == None:
            self.memoryDoors = dict()
        else:
            with open(memoryDoors, "r") as jsonFile:
                self.memoryDoors = json.load(jsonFile)

    def chooseAction(self, state):
        actions = list(self.QTable[state].keys())
        action = ''
        if np.random.uniform(0,1) < self.epsilon:
            action = np.random.choice(actions)
        else:
            # https://thispointer.com/python-how-to-get-all-keys-with-maximum-value-in-a-dictionary/
            # Select the action with highest q_value (if more than one with the same max q_value then select a random one from those)
            maxAction = max(self.QTable[state].items(), key=lambda x: x[1])
            listofmaxActions = [k for k, v in self.QTable[state].items() if v == maxAction[1]]
            action = np.random.choice(listofmaxActions)
        return action

    def getState(self, state, checkingGoal=False):
        if str(state) in self.QTable.keys():
            return str(state)

        x,z,ori = state
        # Create all possible combinations within margin
        all_states = []

        cur_ori = abs(ori)*180/math.pi
        # When checking the goal position consider square area of 6x6
        if checkingGoal:
            for i in range(-6,7):
                for j in range(-6,7):
                    all_states.append(str((round(x+i/100,2),round(z+j/100,2))))
        else:
            # Cases where robot is facing up or down -> x represents corridor width
            if cur_ori < 20 or cur_ori > 160: 
                for i in range(-8,9):
                    for l in range(-20, 21):
                        all_states.append(str((round(x+i/100,2), round(z,2), round(abs(ori+(l*math.pi/180)),1))))
            # Cases where robot is facing sides -> z represents corridor width
            elif cur_ori > 70 and cur_ori < 110:
                for j in range(-8,9):
                    for l in range(-20, 21):
                        all_states.append(str((round(x,2), round(z+j/100,2), round(ori+(l*math.pi/180),1))))

        for s in all_states:
            if s in self.QTable.keys():
                return s

        return ''

    def maxQ(self, state):
        highest_q = 0
        for a in self.QTable[state]:
            nxt_q = self.QTable[state][a]
            if nxt_q >= highest_q:
                highest_q = nxt_q
        return highest_q

    def reachedGoal(self, state):
        tableState = self.getState(state, checkingGoal=True)
        if tableState == self.goal:
            return True
        return False

    def getDoorName(self, state):
        return self.statesNames[state]

    def saveQTable(self, name):
        with open(name, "w+") as jsonFile:
            json.dump(self.QTable, jsonFile)

    def saveMemory(self, name):
        with open(name, "w+") as jsonFile:
            json.dump(self.memoryDoors, jsonFile)

    ###########################################################
    # All functions below this are just used in robot trainig #
    ###########################################################
    def reward(self, state, timePassed):
        # Negative RWD penalizing time spent in low level (to achieve shortest path)
        # High RWD when reaching goal
        if state == self.goal:
            return 100
        else:
            return round(-500*(timePassed/3600),4)
        
    def updateQTable(self, cur_state, next_state, action, timePassed):
        rwd = self.reward(next_state, timePassed)
        cur_q = self.QTable[cur_state][action]
        new_q = cur_q+self.alpha*(rwd+self.gamma*self.maxQ(next_state)-cur_q)
        self.QTable[cur_state][action] = new_q
        print("STATE:", cur_state, " N STATE:", next_state, " Action:", action, " RWD:", rwd, " OldQ:", cur_q, " NewQ:", new_q)

