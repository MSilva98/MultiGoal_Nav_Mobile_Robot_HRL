import json
import itertools
import numpy as np
import matplotlib.pyplot as plt

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


# f1 = open("../Corridor/QTable_v5_6s_wall_bad_consider.txt")
# f2 = open("QTable_corridor_transform.txt")

f1 = open("../Corridor/QTable_v5_6s.txt")
f2 = open("QTable_v7_6states.txt")

QTable1 = json.load(f1)
QTable2 = json.load(f2)

nr_sensors = 6
sensor_states = 6

new_Table = dict()
for state in itertools.product(range(1,sensor_states+1), repeat=nr_sensors):
    state = str(state)
    if not all_zero(list(QTable1[state].values())) and not all_zero(list(QTable2[state].values())):
        act1 = chooseAction(QTable1, state)
        act2 = chooseAction(QTable2, state)
        if act1 != act2:
            new_Table[str(state)] = (act1, act2) 
    
f3 = open("overlaped_states.txt", "w+")
json.dump(new_Table, f3)

f3 = open("overlaped_states.txt", "r")
overlaped_states = json.load(f3)
f4 = open("statesPos.txt", "r")
statesPos = json.load(f4)

overlaped_states_pos = dict()
for s in overlaped_states:
    if s in statesPos:
        if s not in overlaped_states_pos:
            tmp = statesPos[s]
        else:
            tmp = overlaped_states_pos[s] 
            tmp += statesPos[s]
        
        overlaped_states_pos[s] = tmp
        
with open("overlaped_states_pos.txt", "w+") as f5:
    for s in overlaped_states_pos:
        f5.write(s + "\nPOS:" + str(overlaped_states_pos[s])+"\n\n")


# mat = np.zeros((23,23))
# for s in overlaped_states:
#     state = tuple(int(x) for x in s.replace('(','').replace(')','').replace(' ', '').split(','))
#     state = state[0]*100000+state[1]*10000+state[2]*1000+state[3]*100+state[4]*10+state[5]
#     if s in statesPos:
#         for p in statesPos[s]:
#             print(p)
#             mat[int(p[1]*100)+11, int(p[0]*100)+11] = state


# fig, plt1 = plt.subplots(1,figsize=(20,10))
# plt1.matshow(mat, cmap='gist_rainbow', vmin=111111, vmax=666666)
# for i in range(len(mat)):
#     for j in range(len(mat[0])):
#         plt1.text(i, j, int(mat[j, i]), ha="center", va="center", color="black", fontsize='x-small')
# plt1.set(xlabel="X", ylabel="Y")
# plt1.xaxis.set_label_position("top")
# plt.xticks(np.arange(0,23,1))
# plt.yticks(np.arange(0,23,1))
# plt.tight_layout()
# plt.show()
