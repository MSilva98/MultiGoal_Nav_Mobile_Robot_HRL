import json
import itertools
import numpy as np

# Used to join Right Action Table on FR Door and LR Door
# And LR Door entrance with Corridor table

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

# Right FR and LR
# f1 = open("./v52/QTable_v5_17500h_FR_Right.txt", "r")
# f2 = open("./v52/QTable_v5_17500h_LR_Right.txt", "r")
# f3 = open("./v52/QTable_right.txt", "w+")

# Join FR_LR Table with Corner Table
# f1 = open("./v52/QTable_right.txt", "r")
# f2 = open("./v52/QTable_corner_v5_17500h_right.txt", "r")
# f3 = open("./v52/QTable_right_all.txt", "w+")

# f1 = open("./v52/QTable_v5_17500h_FR_Front.txt", "r")
# f2 = open("../Corridor/QTable_v5_17500h.txt", "r")
# f3 = open("./v52/QTable_corridor.txt", "w+")

f1 = open("QTable_22000h_LR_Front.txt", "r")
f2 = open("QTable_corridor.txt", "r")
f3 = open("QTable_corridor_2.txt", "w+")

table1 = json.load(f1)
table2 = json.load(f2)

joint_table = table1

for state in itertools.product(range(1,sensor_states+1), repeat=nr_sensors):
    state = str(state)
    state1 = table1[state]
    state2 = table2[state]

    # Add actions from table2 that aren't already in table 1
    if not toDiscard(list(state2.values()),-500) and toDiscard(list(state1.values()),-500):
        print(state)
        joint_table[state] = state2

json.dump(joint_table, f3)