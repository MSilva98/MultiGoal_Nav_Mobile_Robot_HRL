from os import stat
import numpy as np
import itertools
import math
import json

class Agent():
    def __init__(self, alpha=0.3, gamma=0.99, episodes=500, epsilon=0.1, statesFile=None, Qtable=None, memoryDoors=None, paths=None):
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

        # Actions the agent can perform
        # Depend on type of door
        # FL Door: Front or Left
        # FR Door: Front or Right
        # LR Door: Left or Right

        # Dictionary of available goals in the map
        self.goals = dict()
        if Qtable == None:
            self.QTable = {}
            for state in self.statesF:
                self.QTable[state] = {}
                if self.statesF[state][:2] == "fr":
                    actions = ["front", "right"]
                elif self.statesF[state][:2] == "fl":
                    actions = ["front", "left"]
                elif self.statesF[state][:2] == "lr":
                    actions = ["left", "right"]
                else:
                    actions = ["front", "back"]
                    self.goals[state] = self.statesF[state]
                for a in actions:
                    self.QTable[state][a] = 0  # each state will have all actions
        else:
            with open(Qtable, "r") as jsonFile:
                self.QTable = json.load(jsonFile)
            for state in self.statesF:
                if self.statesF[state][:2] not in ["fr", "fl", "lr", "flr"]:
                    self.goals[state] = self.statesF[state]

        # Memory from blocked doors
        # When the webots controller terminates all the variables are cleaned
        # Robot won't know a path was blocked in before run... this file saves all doors that lead to blocked paths
        if memoryDoors == None:
            self.memoryDoors = dict()
        else:
            self.memoryDoors = json.load(open(memoryDoors, "r"))

        # Paths between goals
        if paths == None:
            self.paths = dict()
        else:
            self.paths = json.load(open(paths, "r"))
            
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

    def maxQ(self, state):
        highest_q = 0
        for a in self.QTable[state]:
            nxt_q = self.QTable[state][a]
            if nxt_q >= highest_q:
                highest_q = nxt_q
        return highest_q

    def getState(self, state, checkingGoal=False):
        if str(state) in self.QTable.keys():
            return str(state)

        x,z,ori = state
        # Create all possible combinations within margin
        all_states = []

        cur_ori = abs(ori)*180/math.pi
        # When checking the goal position consider square area of 6x6
        if checkingGoal:
            # if cur_ori < 20 or cur_ori > 160: 
            #     x_m = 7
            #     y_m = 4
            # elif cur_ori > 70 and cur_ori < 110:
            #     x_m = 4
            #     y_m = 7
                    
            # for i in range(-x_m,x_m+1):
            #     for j in range(-y_m,y_m+1):
            for i in range(-4,5):
                for j in range(-4,5):
                    for l in range(-20, 21):
                        all_states.append(str((round(x+i/100,2),round(z+j/100,2), round(ori+(l*math.pi/180),1))))
            # Check if state exists and is a goal state
            for s in all_states:
                if s in self.QTable.keys() and s in self.goals.keys():
                    return s

        else:
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
            # Check if state exists and is not a goal state
            for s in all_states:
                if s in self.QTable.keys():
                    return s
        return None

    def reachedGoal(self, state, startGoal):
        # tableState = self.getState(state, checkingGoal=True)
        return state in self.goals and self.goals[state][:-1] != startGoal[:-1]

    def reachedSpecificGoal(self, state, goal):
        tableState = self.getState(state, checkingGoal=True)
        return tableState in self.goals and self.goals[tableState][:-1] == goal[:-1]

    def reachedSecondaryGoal(self, state, startGoal, finishGoal):
        tableState = self.getState(state, checkingGoal=True)
        return tableState in self.goals and self.goals[tableState][:-1] != startGoal[:-1] and self.goals[tableState][:-1] != finishGoal[:-1]

    def getGoalName(self, state):
        return self.goals[state]

    def getGoalState(self, goal):
        return [i[0] for i in self.goals.items() if i[1] == goal][0]

    def getDoorState(self, door):
        return [i[0] for i in self.statesF.items() if i[1] == door][0]

    def getRandomGoal(self, goal=None):
        if goal != None:
            availableGoals = [g for g in self.goals.values() if g[:-1] != goal[:-1]]
        else:
            availableGoals = list(self.goals.values())
        return np.random.choice(availableGoals)

    def getDoorName(self, state):
        return self.statesF[state]

    def saveQTable(self, name):
        with open(name, "w+") as jsonFile:
            json.dump(self.QTable, jsonFile)

    def saveMemory(self, name):
        with open(name, "w+") as jsonFile:
            json.dump(self.memoryDoors, jsonFile)

    ###########################################################
    # All functions below this are just used in robot trainig #
    ###########################################################
    def reward(self, state, timePassed, finishGoal):
        # Negative RWD penalizing time spent in low level (to achieve shortest path)
        # High RWD when reaching goal
        if state in self.goals and self.goals[state] == finishGoal:
            return 100
        else:
            return round(-500*(timePassed/3600),4)
        
    def updateQTable(self, cur_state, next_state, action, timePassed, finishGoal):
        rwd = self.reward(next_state, timePassed, finishGoal)
        cur_q = self.QTable[cur_state][action]
        new_q = cur_q+self.alpha*(rwd+self.gamma*self.maxQ(next_state)-cur_q)
        self.QTable[cur_state][action] = new_q
        print("STATE:", cur_state, " N STATE:", next_state, " Action:", action, " RWD:", rwd, " OldQ:", cur_q, " NewQ:", new_q)

