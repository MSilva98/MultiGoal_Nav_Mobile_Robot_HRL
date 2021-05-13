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
        p = (round((float(l[0])+0.85)*100,0), round((float(l[1])+0.85)*100,0))
        if p == (85.0,85.0):
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
    # Right Corner
    # if name == "right_corner" or True:
    y0 = range(0, 51)
    x0 = [0]*len(y0)
    xm15 = range(-15, 51)
    ym15 = [-15]*len(xm15)
    x15 = range(15, 51)
    y15 = [15]*len(x15)
    x2 = range(51)
    y2 = [0]*len(x2)
    y3 = range(-15,51)
    x3 = [-15]*len(y3)
    # Draw line of optimal position vertical
    plt.plot(x0,y0, "k:", label="Optimal Position")
    # Draw line of optimal position horizontal
    plt.plot(x2,y2, "k:")
    # Draw vertical line in x = 15 from y = 15 to y = 50
    plt.plot(y15,x15, "k-")
    # Draw vertical line in x = -15 from y = -15 to y = 50
    plt.plot(x3,y3, "k-")
    # Draw vertical line in y = -15 from x = -15 to x = 50
    plt.plot(xm15,ym15, "k-")
    # Draw horizontal line in y = 15 from x = 15 to x = 50
    plt.plot(x15,y15, "k-")

    plt.xlabel("Robot X (cm)")
    plt.ylabel("Robot Y (cm)")
    plt.xlim(-50, 50)
    if name == "corner_right" or name == "corner_right_17500":
        plt.title("Robot trajectory on corner to the right")
        plt.gca().invert_yaxis()    
    elif name == "corner_left" or name == "corner_left_17500":
        plt.title("Robot trajectory on corner to the left")
        plt.gca().invert_yaxis()    
        plt.gca().invert_xaxis()

    plt.legend()


# plotTrajectory("corner_left")
# plotTrajectory("corner_right")

plotTrajectory("corner_left_17500")
plotTrajectory("corner_right_17500")

plt.tight_layout()
plt.show()