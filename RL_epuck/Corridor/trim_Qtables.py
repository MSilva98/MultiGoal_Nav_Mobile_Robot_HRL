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

    json.dump(trimmedTable, open(name+"_TRIMMED.txt", "w+"))

names = ["./v4/QTable_v5_5000h_FL_Left_z21","./v4/QTable_v5_5000h_FR_Right_Symmetry_z21", "./v4/QTable_v5_13000h_LR_Left_z21","./v4/QTable_v5_13000h_LR_Right_Symmetry","./v4/QTable_v5_13000h_FR_Front_all"]

for n in names:
    trimmTable(n)