import matplotlib.pyplot as plt
from agent import Agent
import numpy as np

x_space = [x/100 for x in range(-11, 12)]       # x = -0.11 to 0.11
y_space = [y/100 for y in range(-11, 12)]       # y = -0.11 to 0.11
ori_space = [o/100 for o in range(-157, 158)]   # o = -1.57 to 1.57

a = Agent()

X, Y = np.meshgrid(x_space, y_space)
Z = []
for y in y_space:
    Z.append([a.reward(x,y,-1.57) for x in x_space])
Z = np.array(Z)
fig = plt.figure()
# ax = fig.add_subplot(2,2,1,projection='3d')
ax = fig.gca(projection='3d')
ax.plot_surface(X,Y,Z, cmap='rainbow')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('RWD')
ax.set_title("Theta = -90º")
plt.tight_layout()
plt.savefig("theta_m90.png")

Z = []
for y in y_space:
    Z.append([a.reward(x,y,1.57) for x in x_space])
Z = np.array(Z)
fig = plt.figure()
# ax = fig.add_subplot(2,2,2,projection='3d')
ax = fig.gca(projection='3d')
ax.plot_surface(X,Y,Z, cmap='rainbow')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('RWD')
ax.set_title("Theta = 90º")
plt.tight_layout()
plt.savefig("theta_90.png")

Z = []
for y in y_space:
    Z.append([a.reward(x,y,-0.7408) for x in x_space])
Z = np.array(Z)
fig = plt.figure()
# ax = fig.add_subplot(2,2,3,projection='3d')
ax = fig.gca(projection='3d')
ax.plot_surface(X,Y,Z, cmap='rainbow')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('RWD')
ax.set_title("Theta = -45º")
plt.tight_layout()
plt.savefig("theta_m45.png")

Z = []
for y in y_space:
    Z.append([a.reward(x,y,0.7408) for x in x_space])
Z = np.array(Z)
fig = plt.figure()
# ax = fig.add_subplot(2,2,4,projection='3d')
ax = fig.gca(projection='3d')
ax.plot_surface(X,Y,Z, cmap='rainbow')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('RWD')
ax.set_title("Theta = 45º")
plt.tight_layout()
plt.savefig("theta_45.png")

# plt.tight_layout()
# plt.show()


