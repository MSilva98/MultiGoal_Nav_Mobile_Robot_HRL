import matplotlib.pyplot as plt

font_size = 20

def plotTrajectory(name):
    f = open(name+".txt", "r")
    xy = []
    tmp_x = []
    tmp_y = []
    for line in f:
        l = line.replace('(','').replace(')','').replace(' ', '').split(',')
        p = (round(float(l[0])*100,0), round(float(l[1])*100,0))
        if p == (-500.0,-500.0) and len(tmp_y) != 0:
            xy.append((tmp_x, tmp_y))
            tmp_x = []
            tmp_y = []
        elif p != (-500.0, -500.0):
            tmp_x.append(p[0])
            tmp_y.append(p[1])

    y = range(-150, 151)
    x1 = [0]*len(y)
    xm15 = [-15]*len(y)
    x15 = [15]*len(y)
    xL = range(-15, 16)
    yTop = [150]*len(xL)
    yBot = [-150]*len(xL)
    fig3 = plt.figure()
    plt.plot(xy[0][1],xy[0][0], "r-", label="Robot path")
    xy.pop(0)
    for v in xy:
        plt.plot(v[1],v[0], "r-")
    
    plt.plot(y,x1, "k:", label="Optimal Position")
    plt.plot(y,xm15, "k-")
    plt.plot(y,x15, "k-")
    plt.plot(yTop,xL, "k-")
    plt.plot(yBot,xL, "k-")
    plt.xlabel("Robot X (cm)", fontsize=font_size)
    plt.ylabel("Robot Y (cm)", fontsize=font_size)
    if name == "agent_positions":
        plt.title("Robot Trajectory in Corridor", fontsize=font_size)
        plt.xlim(-155, 155)
    if name == "agent_convergence":
        plt.title("Robot Convergence in Corridor", fontsize=font_size)
        plt.xlim(50, 110)

    plt.xticks(fontsize=font_size)
    plt.yticks(range(-15, 16), fontsize=font_size)
    plt.legend(fontsize=font_size)
    plt.tight_layout()


plotTrajectory("agent_positions")
plotTrajectory("agent_convergence")

plt.show()