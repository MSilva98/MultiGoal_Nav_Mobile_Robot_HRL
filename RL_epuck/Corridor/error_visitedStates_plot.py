import matplotlib.pyplot as plt
import numpy as np
import json

f = open("agent_positions.txt", "r")
pos = []
for line in f:
    pos.append(float(line)*100)

x = range(len(pos))
y = [0]*len(pos)

fig = plt.figure()
plt.plot(x,pos, "rx:", label="Real Position")
plt.plot(x,y, "k:", label="Optimal Position")
plt.xlabel("Time (s)")
plt.ylabel("Robot X axis (cm)")
plt.title("Robot position in corridor")
plt.yticks(range(-11, 12))
plt.legend()

# f = open("visited_states.txt", "r")
# visited_states = json.load(f)

# fig2 = plt.figure()
# plt.bar(range(len(visited_states)),list(visited_states.values()))
# plt.xlabel("State")
# plt.ylabel("Times visited")
# plt.title("Robot Visited States")
# plt.xticks(range(len(visited_states)), list(visited_states.keys()), rotation=90)

f = open("agent_convergence.txt", "r")
xy = []
tmp_x = []
tmp_y = []
for line in f:
    l = line.replace('(','').replace(')','').replace(' ', '').split(',')
    p = (round(float(l[0])*100,0), round(float(l[1])*100,0))
    if p == (0.0,0.0) and len(tmp_y) != 0:
        xy.append((tmp_x, tmp_y))
        tmp_x = []
        tmp_y = []
    else:
        tmp_x.append(p[0])
        tmp_y.append(p[1])

min_y = int(min(xy[0][1]))
max_y = int(max(xy[0][1]))
for v in xy:
    if int(min(v[1])) < min_y:
        min_y = int(min(v[1]))
    if int(max(v[1])) > max_y:
        max_y = int(max(v[1]))

y1 = range(min_y, max_y)
x1 = [0]*len(y1)
fig3 = plt.figure()
for v in xy:
    plt.plot(v[0],v[1], "r-")
plt.plot(x1,y1, "k:", label="Optimal Position")
plt.xlabel("Robot X (cm)")
plt.ylabel("Robot Y (cm)")
plt.title("Robot convergence in corridor")
plt.xticks(range(-11, 12))
plt.legend()



plt.tight_layout()
plt.show()