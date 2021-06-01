import json
import itertools
import numpy as np

# Create a symmetric table from the one given
# Used to get Turn Left on Door (FL or LR) Table  

def symmetricAction(action):
    if action == "F":
        return "F"
    elif action == "LL":
        return "LR"
    elif action == "LR":
        return "LL"
    elif action == "ML":
        return "MR"
    elif action == "MR":
        return "ML"
    elif action == "L":
        return "R"
    elif action == "R":
        return "L"
    elif action == "HL":
        return "HR"
    elif action == "HR":
        return "HL"
    else:   # Bad Action -> SHOULD NEVER HAPPEN
        return -1

# Sensor symmetry
# FL becomes FR
# L becomes R
def symmetricState(state):
    return (state[0], state[3], state[4], state[1], state[2], state[5])

def applySymmetry(dct):
    newDict = dict()
    for a in dct:
        symAct = symmetricAction(a)
        newDict[symAct] = dct[a]
    return newDict

nr_sensors = 6
sensor_states = 6

actions = ["F", "LL", "LR", "ML", "MR", "L", "R", "HL", "HR"]

# originalTable = open("../LowLevelCombined/QTable_Right_Door.txt", "r")
# symmetricTable = open("../LowLevelCombined/QTable_Left_Door.txt", "w+")

# originalTable = open("./v52/QTable_v5_17500h_LR_Right.txt", "r")
# symmetricTable = open("./v52/QTable_v5_17500h_LR_Left.txt", "w+")

# originalTable = open("./v52/QTable_v5_17500h_FR_Right.txt", "r")
# symmetricTable = open("./v52/QTable_v5_17500h_FL_Left.txt", "w+")

originalTable = open("QTable_22000h_FR_Right.txt", "r")
symmetricTable = open("QTable_22000h_FR_Left.txt", "w+")


print("Creating symmetric table...")
oriTable = json.load(originalTable)
symTable = dict()
for state in itertools.product(range(1,sensor_states+1), repeat=nr_sensors):
    oriState = str(state)
    symState = str(symmetricState(state))
    oriActions = oriTable[oriState]
    symActions = applySymmetry(oriActions)
    symTable[symState] = symActions
json.dump(symTable, symmetricTable)
print("Symmetric table complete!")
