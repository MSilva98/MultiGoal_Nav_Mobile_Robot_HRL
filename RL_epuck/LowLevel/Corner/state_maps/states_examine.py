import json
import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.widgets import Slider, Button, RadioButtons

# This funcion plots all states for X,Y in a given rotation
def plot_map(name):
    print(name)
    states = []
    with open(name+".txt", "r") as jsonFile:
        states = json.load(jsonFile)
    mat = np.zeros((23, 23))
    diff_states = []
    pos = []
    for k in states:
        # Convert from string to integer
        state = tuple(int(x) for x in k.replace('(','').replace(')','').replace(' ', '').split(','))
        state = state[0]*100000+state[1]*10000+state[2]*1000+state[3]*100+state[4]*10+state[5]
        for p in states[k]:
            p = (int((p[0])*100)+11, int((p[1])*100)+11, p[0], p[1])
            # if p[2] == 0:
            if state not in diff_states:
                diff_states.append(state)
            if mat[p[1], p[0]] == 0:
                pos.append(p)
                mat[p[1], p[0]] = state

    fig, plt1 = plt.subplots(1,figsize=(20,10))
    plt1.matshow(mat, cmap='tab20', vmin=111111, vmax=666666)
    for i in range(len(mat)):
        for j in range(len(mat[0])):
            plt1.text(i, j, int(mat[j, i]), ha="center", va="center", color="black", fontsize='x-small')
    plt1.set(xlabel="X", ylabel="Y")
    plt1.xaxis.set_label_position("top")
    plt.xticks(np.arange(0,23,1))
    plt.yticks(np.arange(0,23,1))
    plt.tight_layout()
    # plt.show()
    fig.savefig("6s_"+name+".png")

# for x in range(0,360, 10):
#     plot_map("6_states_"+str(float(x)))
# plot_map("6_states_0.0")


# Does the same as the previous one but overlaps all in one plot
def posStates(name):
    all_pos_by_state = dict()
    all_states_by_pos = dict()
    states = []
    for x in range(0,360, 10):
        with open(name+str(float(x))+".txt", "r") as jsonFile:
            states = json.load(jsonFile)
        for s in states:
            for p in states[s]:
                p = tuple(p)
                if p not in all_states_by_pos:
                    all_states_by_pos[p] = [s]
                else:
                    all_states_by_pos[p].append(s)
            if s not in all_pos_by_state:
                tmp = []
                for p in states[s]:
                    # tmp.append((round(p[0]*100,0),round(p[2]*180/math.pi,0)))                
                    tmp.append((round(p[0]*100,0),round(p[1]*100,0),round(p[2]*180/math.pi,0)))                
            else:
                tmp = all_pos_by_state[s]
                for p in states[s]:
                    # tmp.append((round(p[0]*100,0),round(p[2]*180/math.pi,0)))
                    tmp.append((round(p[0]*100,0),round(p[1]*100,0),round(p[2]*180/math.pi,0)))
            all_pos_by_state[s] = tmp

    # mat = np.zeros((23,23))
    # for s in all_pos_by_state:
    #     state = tuple(int(x) for x in s.replace('(','').replace(')','').replace(' ', '').split(','))
    #     state = state[0]*100000+state[1]*10000+state[2]*1000+state[3]*100+state[4]*10+state[5]
    #     for p in all_pos_by_state[s]:
    #         mat[int(p[1])+11, int(p[0])+11] = state        
    # _, plt1 = plt.subplots(1,figsize=(20,10))
    # plt1.matshow(mat, cmap='tab20', vmin=111111, vmax=666666)
    # for i in range(len(mat)):
    #     for j in range(len(mat[0])):
    #         plt1.text(i, j, int(mat[j, i]), ha="center", va="center", color="black", fontsize='x-small')
    # plt1.set(xlabel="X", ylabel="Y")
    # plt1.xaxis.set_label_position("top")
    # plt.xticks(np.arange(0,23,1))
    # plt.yticks(np.arange(0,23,1))
    # plt.tight_layout()
    # plt.show()

    num_states = dict()
    for s in all_pos_by_state:
        if s not in num_states:
            num_states[s] = len(all_pos_by_state[s])

    f = open("6s_num_states.txt", "w+")
    for n in num_states:
        f.write(n + " -> " + str(num_states[n]) + "\nPOS: " + str(all_pos_by_state[n]) + "\n\n")

posStates("6_states_")