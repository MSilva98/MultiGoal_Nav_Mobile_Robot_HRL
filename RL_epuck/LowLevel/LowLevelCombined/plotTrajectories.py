import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

font_size = 20

def plotTrajectories(name):
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
        elif p != (-500.0,-500.0):
            tmp_x.append(p[0])
            tmp_y.append(p[1])

    fig = plt.figure()
    # Draw Maze
    # External Walls
    ex = range(-100, 101)
    ey = [-100]*len(ex)
    ey1 = [100]*len(ex)
    plt.plot(ex, ey, "k-", linewidth=3)
    plt.plot(ex, ey1, "k-", linewidth=3)
    plt.plot(ey, ex, "k-", linewidth=3)
    plt.plot(ey1, ex, "k-", linewidth=3)
    # Middle walls
    rect1 = Rectangle((-70,-70),140,55,linewidth=1,edgecolor='k',facecolor='k')
    rect2 = Rectangle((-70,15),140,55,linewidth=1,edgecolor='k',facecolor='k')
    plt.gca().add_patch(rect1)
    plt.gca().add_patch(rect2)
    
    # Plot trajectories
    plt.plot(xy[0][0],xy[0][1], "r-", label="Robot Trajectory")
    xy.pop(0)
    for v in xy:
      plt.plot(v[0],v[1], "r-")

    if name == "trajectory_front":
      plt.title("Robot trajectory on Maze 8 going forward on doors", fontsize=font_size)
    elif name == "trajectory_right":
      plt.title("Robot trajectory on Maze 8 going right on doors", fontsize=font_size)
    elif name == "trajectory_left":
      plt.title("Robot trajectory on Maze 8 going left on doors", fontsize=font_size)
    
    plt.xlabel("Robot X (cm)", fontsize=font_size)
    plt.ylabel("Robot Y (cm)", fontsize=font_size)
    plt.xticks(fontsize=font_size)
    plt.yticks(fontsize=font_size)
    plt.xlim(-105, 150)
    plt.ylim(-105, 105)
    plt.gca().invert_yaxis()
    plt.legend(loc="upper right", fontsize=font_size)
    plt.tight_layout()

    
plotTrajectories("trajectory_front")
plotTrajectories("trajectory_right")
plotTrajectories("trajectory_left")

plt.show()
