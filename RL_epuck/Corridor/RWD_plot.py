import matplotlib.pyplot as plt
from agent import Agent
import numpy as np

x_space = [x/100 for x in range(-11, 12)]       # x = -0.11 to 0.11
ori_space = [o/100 for o in range(-314, 315)]   # o = -1.57 to 1.57

def reward(pos, ori):
    if (pos >= 0 and ori >= 0) or (pos <= 0 and ori <= 0):  # Cases when robot is moving to the center
        return -400*(10*pow(pos,2)+0.02*pow(ori,2))+15
    else: # Cases when it is walking away
        return -400*(15*pow(pos,2)+0.03*pow(ori,2))-10

X, Y = np.meshgrid(x_space, ori_space)

Z = []
for o in ori_space:
    Z.append([reward(x,o) for x in x_space])

Z = np.array(Z)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X,Y,Z, cmap='rainbow')
ax.set_xlabel('X (m)')
ax.set_ylabel('Theta (rads)')
ax.set_zlabel('RWD')
ax.set_title("Corridor Reward")

plt.tight_layout()
plt.show()


