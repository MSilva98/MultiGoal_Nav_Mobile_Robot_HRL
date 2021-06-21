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
        elif p != (-500.0,-500.0):
            tmp_x.append(p[0])
            tmp_y.append(p[1])

    fig3 = plt.figure()
    # Draw corridor and doors
    if name == "doors_trajectory":
        # Plot one path to create label INVERTED
        plt.plot(xy[0][1],xy[0][0], "r-", label="Robot path")
        xy.pop(0)
        for v in xy:
            plt.plot(v[1],v[0], "r-")
        
        y0 = range(0, 101)
        x0 = [0]*len(y0)
        y15down = range(-100, -14)
        y15up  = range(15, 101)
        x15up = [15]*len(y15up)
        x15down = [15]*len(y15down)
        xm15up = [-15]*len(y15up)
        x1 = range(-100, 101)
        ym15 = [-15]*len(x1)
        y2 = [0]*len(x1)

        xm15_15 = range(-15, 16)
        ym100 = [-100]*len(xm15_15)
        y100 = [100]*len(xm15_15)

        plt.plot(xm15_15,y100, "k-")
        plt.plot(y100,xm15_15, "k-")
        plt.plot(ym100,xm15_15, "k-")

        plt.plot(x0,y0, "k:", label="Optimal Position")
        plt.plot(x1,y2, "k:")
        plt.plot(xm15up,y15up, "k-")
        plt.plot(x15up,y15up, "k-")
        plt.plot(y15up,x15up, "k-")
        plt.plot(y15down,x15down, "k-")
        plt.plot(x1,ym15, "k-")
        plt.xlabel("Robot X (cm)", fontsize=font_size)
        plt.ylabel("Robot Y (cm)", fontsize=font_size)
        plt.xticks(fontsize=font_size)
        plt.yticks(fontsize=font_size)
        plt.xlim(-101, 101)
        plt.title("Robot trajectory on doors", fontsize=font_size)
        plt.gca().invert_yaxis()    
        plt.legend(fontsize=font_size)
    else:
        # Plot one path to create label
        plt.plot(xy[0][0],xy[0][1], "r-", label="Robot path")
        xy.pop(0)
        for v in xy:
            plt.plot(v[0],v[1], "r-")
        # Front Left and Front Right doors
        if name == "door_right_fr" or name == "door_left_fl" or name == "door_right_fr_fwd" or name == "door_left_fl_fwd":
            y0 = range(-60, 61)
            y15down = range(-60, -14)
            y15up  = range(15, 61)
            x0 = [0]*len(y0)
            x15 = [15]*len(y15down)
            xm15 = [-15]*len(y0)
            x1 = range(15, 61)
            ym15 = [-15]*len(x1)
            y15  = [15]*len(x1)
            x2 = range(61)
            y2 = [0]*len(x2)
            plt.plot(x0,y0, "k:", label="Optimal Position")
            plt.plot(x2,y2, "k:")
            plt.plot(x15,y15up, "k-")
            plt.plot(x15,y15down, "k-")
            plt.plot(xm15,y0, "k-")
            plt.plot(x1,ym15, "k-")
            plt.plot(x1,y15, "k-")
        # Left Right doors
        else:
            y0 = range(0, 61)
            x0 = [0]*len(y0)
            y15down = range(-60, -14)
            y15up  = range(15, 61)
            x15up = [15]*len(y15up)
            x15down = [15]*len(y15down)
            xm15up = [-15]*len(y15up)
            x1 = range(-60, 61)
            ym15 = [-15]*len(x1)
            y2 = [0]*len(x1)
            plt.plot(x0,y0, "k:", label="Optimal Position")
            plt.plot(x1,y2, "k:")
            plt.plot(xm15up,y15up, "k-")
            plt.plot(x15up,y15up, "k-")
            plt.plot(y15up,x15up, "k-")
            plt.plot(y15down,x15down, "k-")
            plt.plot(x1,ym15, "k-")

        plt.xlabel("Robot X (cm)")
        plt.ylabel("Robot Y (cm)")
        plt.xlim(-61, 61)
        plt.legend()
        if name == "door_right_fr" or name == "door_right_lr":
            plt.title("Robot trajectory on door to the right")
            plt.gca().invert_yaxis()    
        elif name == "door_left_fl":
            plt.title("Robot trajectory on door to the left")
            plt.gca().invert_xaxis()
        elif name == "door_left_lr":
            plt.title("Robot trajectory on door to the left")
            plt.gca().invert_yaxis()    
        elif name == "door_right_fr_fwd":
            plt.title("Robot trajectory on right door forward")
            plt.gca().invert_yaxis()
        elif name == "door_left_fl_fwd":
            plt.title("Robot trajectory on left door forward")
            plt.gca().invert_xaxis()   
    
    plt.tight_layout()
    

# Trajectory for each door
plotTrajectory("door_right_fr")
plotTrajectory("door_left_fl")
plotTrajectory("door_right_fr_fwd")
plotTrajectory("door_left_fl_fwd")
plotTrajectory("door_right_lr")
plotTrajectory("door_left_lr")

# Trajectory with all doors combined
plotTrajectory("doors_trajectory")

plt.show()