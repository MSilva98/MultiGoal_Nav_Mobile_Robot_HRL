import matplotlib.pyplot as plt
import numpy as np
import json

def plotTrajectory(name):
    f = open(name+".txt", "r")
    xy = []
    tmp_x = []
    tmp_y = []
    for line in f:
        l = line.replace('(','').replace(')','').replace(' ', '').split(',')
        p = (round(float(l[0])*100,0), round(float(l[1])*100,0))
        if p == (0.0,0.0):
            if len(tmp_y) != 0:
                xy.append((tmp_x, tmp_y))
                tmp_x = []
                tmp_y = []
        else:
            tmp_x.append(p[0])
            tmp_y.append(p[1])

    fig3 = plt.figure()
    # Plot one path to create label
    plt.plot(xy[0][0],xy[0][1], "r-", label="Robot path")
    xy.pop(0)
    for v in xy:
        plt.plot(v[0],v[1], "r-")

    # Draw corridor and corner
    y0 = range(-85, 86)
    xm0  = [-85]*len(y0)
    x0   = [85]*len(y0)
    xm100 = range(-100, 101)
    ym100 = [-100]*len(xm100)
    y100 = [100]*len(xm100)
    xm70 = range(-70, 71)
    ym70 = [-70]*len(xm70)
    y70 = [70]*len(xm70)
    
    # Draw line of optimal position
    plt.plot(x0,y0, "k:", label="Corridor Mid Line")
    plt.plot(xm0,y0, "k:")
    plt.plot(y0,xm0, "k:")
    plt.plot(y0,x0, "k:")
    # Draw exterior walls
    plt.plot(y100,xm100, "k-")
    plt.plot(ym100,xm100, "k-")
    plt.plot(xm100,ym100, "k-")
    plt.plot(xm100,y100, "k-")
    # Draw interior walls
    plt.plot(y70,xm70, "k-")
    plt.plot(ym70,xm70, "k-")
    plt.plot(xm70,ym70, "k-")
    plt.plot(xm70,y70, "k-")

    plt.xlabel("Robot X (cm)")
    plt.ylabel("Robot Y (cm)")
    plt.xlim(-105, 105)
    plt.ylim(-105, 105)
    if name == "corner_right" or name == "corner_right_17500_all":
        plt.title("Robot trajectory on corner to the right")    
        plt.arrow(0, 93, 20, 0, length_includes_head=True,
          head_width=5, head_length=5, label="Robot Direction", color="blue")
    elif name == "corner_left" or name == "corner_left_17500_all":
        plt.title("Robot trajectory on corner to the left")
        plt.arrow(0, 93, -20, 0, length_includes_head=True,
          head_width=5, head_length=5, label="Robot Direction", color="blue")

    plt.legend()


plotTrajectory("corner_left")
plotTrajectory("corner_right")


plt.tight_layout()
plt.show()