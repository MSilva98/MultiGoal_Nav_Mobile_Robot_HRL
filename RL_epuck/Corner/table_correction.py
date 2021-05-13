import json

# Some tables were wrongly initiated with zero 
# They are converted using this algorithm were every action with value zero changes to -500
f = open("QTable_corridor_v5_6500h_left_symmetric.txt")

qtable = json.load(f)

for state in qtable:
    for action in qtable[state]:
        if qtable[state][action] == 0:
            qtable[state][action] = -500

json.dump(qtable, open("QTable_corner_left_sym_corrected.txt", "w+"))
