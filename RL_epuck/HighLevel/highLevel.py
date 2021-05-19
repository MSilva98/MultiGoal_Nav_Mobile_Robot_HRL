from os import stat
import numpy as np
import itertools
import math
import json

class Agent():
    def __init__(self, alpha=0.3, gamma=0.99, episodes=500, epsilon=0.1, statesFile=None, Qtable=None):
        self.alpha = alpha
        self.gamma = gamma
        self.episodes = episodes
        self.epsilon = epsilon

        # States -> Will be each door entrance - DEPEND ON EACH MAZE
        # Each maze has a statesFile with dictionary of type -> {"(x, z)": "doorType",}
        # Last position in dictionary defines goal -> "(x, z)": "goal"
        # NOTE: The space between x and z in (x, z) is MANDATORY
        if statesFile == None:
            print("MUST DEFINE STATES BEFORE")
        else:
            with open(statesFile, "r") as jsonFile:
                self.statesF = json.load(jsonFile)

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
                # x,z = [round(float(v),2) for v in state.replace('(','').replace(')','').replace(' ','').split(',')]
                # self.states.append((x,z))
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

    def getState(self, state):
        if str(state) in self.QTable.keys():
            return str(state)

        # x,z = [round(float(v),2) for v in state.replace('(','').replace(')','').replace(' ','').split(',')]
        x,z = state
        # Create all possible combinations within margin
        all_states = []
        for i in range(-7,8):
            for j in range(-3,4):
                all_states.append(str((round(x+i/100,2),round(z+j/100,2))))

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
        tableState = self.getState(state)
        if tableState == '':
            return False
        if tableState == self.goal:
            return True
        return False

    ###########################################################
    # All functions below this are just used in robot trainig #
    ###########################################################
    def reward(self, state, timePassed):
        # Negative RWD penalizing time spent in low level (to achieve shortest path)
        # High RWD when reaching goal
        if state == self.goal:
            return 100
        else:
            return round(-10*(timePassed/3600),4)
        
    def updateQTable(self, cur_state, next_state, action, timePassed):
        rwd = self.reward(next_state, timePassed)
        cur_q = self.QTable[cur_state][action]
        new_q = cur_q+self.alpha*(rwd+self.gamma*self.maxQ(next_state)-cur_q)
        self.QTable[cur_state][action] = new_q
        print("STATE:", cur_state, " N STATE:", next_state, " Action:", action, " RWD:", rwd)

    def saveQTable(self, name):
        with open(name, "w+") as jsonFile:
            json.dump(self.QTable, jsonFile)

