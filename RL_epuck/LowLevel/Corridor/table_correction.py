import json

# Some tables were wrongly initiated with zero 
# They are converted using this algorithm were every action with value zero changes to -500
f = open("QTable_v5_6s_4epsilon_13000h.txt")

qtable = json.load(f)

for state in qtable:
    for action in qtable[state]:
        if qtable[state][action] == 0:
            qtable[state][action] = -500

json.dump(qtable, open("QTable_v5_13000h_corrected.txt", "w+"))
