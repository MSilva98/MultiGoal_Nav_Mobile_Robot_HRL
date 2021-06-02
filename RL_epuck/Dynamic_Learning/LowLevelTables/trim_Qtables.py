import json
import itertools
import numpy as np

def chooseAction(table, state):
    action = ''
    # https://thispointer.com/python-how-to-get-all-keys-with-maximum-value-in-a-dictionary/
    # Select the action with highest q_value (if more than one with the same max q_value then select a random one from those)
    maxAction = max(table[state].items(), key=lambda x: x[1])
    listofmaxActions = [k for k, v in table[state].items() if v == maxAction[1]]
    action = np.random.choice(listofmaxActions)
    if len(listofmaxActions) > 1:
        print(state, listofmaxActions)
    return action

def toDiscard(lst, value):
    for v in lst:
        if v != value:
            return False
    return True

nr_sensors = 6
sensor_states = 6

def trimmTable(name):
    print(name)
    originalTable = open(name+".txt", "r")
    oriTable = json.load(originalTable)
    trimmedTable = dict()
    for state in itertools.product(range(1,sensor_states+1), repeat=nr_sensors):
        state = str(state)
        actions = oriTable[state].values()
        if not toDiscard(actions, -500):
            bestAction = chooseAction(oriTable, state)
            trimmedTable[state] = bestAction

    json.dump(trimmedTable, open(name+"_trimmed.txt", "w+"))

names = ["QTable_corridor","QTable_left_all", "QTable_right_all"]

for n in names:
    trimmTable(n)