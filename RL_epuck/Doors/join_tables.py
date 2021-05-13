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

# Left (Not really used as symmetry of the result from the Right FR and LR is used)
# f1 = open("./v5/QTable_v5_13000h_corrected_FL_Left_z0_x20_sym.txt", "r")
# f2 = open("./v5/QTable_v5_13000h_corrected_LR_Left_sym.txt", "r")
# f3 = open("./v5/QTable_door_left.txt", "w+")

# Corridor and LR Entrance
# f1 = open("../Corridor/QTable_v5_17500h_corrected.txt", "r")
# f2 = open("../LowLevelCombined/QTable_Front.txt", "r")
# f3 = open("./lr/QTable_corridor.txt", "w+")

# Right FR and LR
f1 = open("./v52/QTable_v5_17500h_FR_Right.txt", "r")
# f1 = open("../LowLevelCombined/QTable_Right_Corner.txt", "r")
f2 = open("../LowLevelCombined/QTable_LR_Right.txt", "r")
f3 = open("./v52/QTable_right.txt", "w+")


table1 = json.load(f2)
table2 = json.load(f1)

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