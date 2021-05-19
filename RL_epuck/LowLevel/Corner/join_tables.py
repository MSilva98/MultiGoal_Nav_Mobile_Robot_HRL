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

def all_zero(lst):
    for v in lst:
        if v != 0:
            return False
    return True


f1 = open("../Corridor/QTable_v5_6s.txt")
f2 = open("QTable_v7_6states.txt")

QTable = json.load(f1)
RwdTable = json.load(f2)

nr_sensors = 6
sensor_states = 6

new_Table = dict()
for state in itertools.product(range(1,sensor_states+1), repeat=nr_sensors):
    state = str(state)
    if not all_zero(list(RwdTable[state].values())): 
        new_Table[str(state)] = chooseAction(RwdTable, state)
    elif not all_zero(list(QTable[state].values())):
        new_Table[str(state)] = chooseAction(QTable, state)
    

f3 = open("joined_table.txt", "w+")
json.dump(new_Table, f3)