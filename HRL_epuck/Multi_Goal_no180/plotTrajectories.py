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
    if name == "maze1_trajectory":
        # Draw Maze
        # Bottom horizontal wall
        hx1 = range(-70,101)
        hy1 = [70]*len(hx1)
        # Top vertical left wall
        vy2 = range(-70, -19)
        vx2 = [-70]*len(vy2)
        # Top horizontal wall
        hx2 = range(-70,71)
        hy2 = [-70]*len(hx2)
        # External Walls
        ex = range(-100, 101)
        ey = [-100]*len(ex)
        ey1 = [100]*len(ex)
        plt.plot(hx1, hy1, "k-", linewidth=2)
        plt.plot(vx2, vy2, "k-", linewidth=2)
        plt.plot(hx2, hy2, "k-", linewidth=2)
        plt.plot(ex, ey, "k-", linewidth=3)
        plt.plot(ex, ey1, "k-", linewidth=3)
        plt.plot(ey, ex, "k-", linewidth=3)
        plt.plot(ey1, ex, "k-", linewidth=3)
        # Middle walls
        rect = Rectangle((-55,-70),30,30,linewidth=1,edgecolor='m',facecolor='m', label="Goal")
        rect1 = Rectangle((-20,10),40,30,linewidth=1,edgecolor='k',facecolor='k')
        rect2 = Rectangle((-70,10),20,60,linewidth=1,edgecolor='k',facecolor='k')
        rect3 = Rectangle((-70,-40),90,20,linewidth=1,edgecolor='k',facecolor='k')
        rect4 = Rectangle((50,-70),20,110,linewidth=1,edgecolor='k',facecolor='k')
        plt.gca().add_patch(rect)
        plt.gca().add_patch(rect1)
        plt.gca().add_patch(rect2)
        plt.gca().add_patch(rect3)
        plt.gca().add_patch(rect4)

        # Plot trajectories
        plt.plot(xy[0][0],xy[0][1], "r-", label="Trajectory 1")
        plt.plot(xy[1][0],xy[1][1], "g-", label="Trajectory 2")
        plt.plot(xy[2][0],xy[2][1], "b-", label="Trajectory 3")
        plt.plot(xy[3][0],xy[3][1], "c-", label="Trajectory 4")
        
        plt.title("Robot trajectory on Maze 1", fontsize=font_size)
        plt.xlabel("Robot X (cm)", fontsize=font_size)
        plt.ylabel("Robot Y (cm)", fontsize=font_size)
        plt.xticks(fontsize=font_size)
        plt.yticks(fontsize=font_size)
        plt.xlim(-105, 160)
        plt.ylim(-105, 105)
        plt.gca().invert_yaxis()
        plt.legend(loc="upper right", fontsize=font_size)
        plt.tight_layout()
    else:
        # Draw Maze
        topLeft = Rectangle((-100,-100),60,30,linewidth=2,edgecolor='k',facecolor='k')
        topMiddle = Rectangle((-10,-100),60,30,linewidth=2,edgecolor='k',facecolor='k')
        topRight = Rectangle((80,-130),20,80,linewidth=2,edgecolor='k',facecolor='k')
        middleRight = Rectangle((80,-20),20,60,linewidth=2,edgecolor='k',facecolor='k')
        middleLeft1 = Rectangle((-100,10),90,30,linewidth=2,edgecolor='k',facecolor='k')
        middleLeft2 = Rectangle((-100,-40),120,20,linewidth=2,edgecolor='k',facecolor='k')
        bottomRight = Rectangle((50,70),50,30,linewidth=2,edgecolor='k',facecolor='k')
        bottomMiddle = Rectangle((-35,70),55,30,linewidth=2,edgecolor='k',facecolor='k')
        bottomLeft = Rectangle((-100,70),35,30,linewidth=2,edgecolor='k',facecolor='k')
        gym = Rectangle((101.5,-110),27,30,linewidth=1,edgecolor='m',facecolor='m',label="Gym")
        home = Rectangle((65,102),27,26,linewidth=1,edgecolor='r',facecolor='r',label="Home")
        restaurant = Rectangle((-128.5,46),27,30,linewidth=1,edgecolor='g',facecolor='g',label="Restaurant")
        college = Rectangle((-80,-128),27,26,linewidth=1,edgecolor='b',facecolor='b',label="College")
        library = Rectangle((21.5,-17),27,30,linewidth=1,edgecolor='c',facecolor='c',label="Library")
        plt.gca().add_patch(topLeft)
        plt.gca().add_patch(topMiddle)
        plt.gca().add_patch(topRight)
        plt.gca().add_patch(middleRight)
        plt.gca().add_patch(middleLeft1)
        plt.gca().add_patch(middleLeft2)
        plt.gca().add_patch(bottomRight)
        plt.gca().add_patch(bottomMiddle)
        plt.gca().add_patch(bottomLeft)
        plt.gca().add_patch(gym)
        plt.gca().add_patch(home)
        plt.gca().add_patch(college)
        plt.gca().add_patch(restaurant)
        plt.gca().add_patch(library)

        # Vertical walls
        # Left
        vy1 = range(10,101)
        vx1 = [-100]*len(vy1)
        # Middle
        vy2 = range(-20,101)
        vx2 = [20]*len(vy2)
        # Right
        if name == "maze2_trajectory_updated":
          vy3 = range(-130, 41)
        else:
          vy3 = range(-70, 41)
        vx3 = [50]*len(vy3)
        # Horizontal wall
        hx1 = range(50,81)
        hy1 = [40]*len(hx1)
        # External Walls
        ex = range(-130, 131)
        ey = [-130]*len(ex)
        ey1 = [130]*len(ex)
        plt.plot(vx1, vy1, "k-", linewidth=2)
        plt.plot(vx2, vy2, "k-", linewidth=2)
        plt.plot(vx3, vy3, "k-", linewidth=2)
        plt.plot(hx1, hy1, "k-", linewidth=2)
        plt.plot(ex, ey, "k-", linewidth=3)
        plt.plot(ex, ey1, "k-", linewidth=3)
        plt.plot(ey, ex, "k-", linewidth=3)
        plt.plot(ey1, ex, "k-", linewidth=3)

        # Plot trajectories
        plt.plot(xy[0][0],xy[0][1], "r-", label="Home-Library")
        plt.plot(xy[1][0],xy[1][1], "c-", label="Library-Gym")
        plt.plot(xy[2][0],xy[2][1], "m-", label="Gym-Restaurant")
        plt.plot(xy[3][0],xy[3][1], "g-", label="Restaurant-Home")

        plt.title("Robot trajectory on Maze 2", fontsize=font_size)
        plt.xlabel("Robot X (cm)", fontsize=font_size)
        plt.ylabel("Robot Y (cm)", fontsize=font_size)
        plt.xticks(fontsize=font_size)
        plt.yticks(fontsize=font_size)
        plt.xlim(-135, 200)
        plt.ylim(-135, 135)
        plt.gca().invert_yaxis()    
        plt.legend(loc="upper right", fontsize=font_size)
        plt.tight_layout()

    
# plotTrajectories("maze1_trajectory")
plotTrajectories("maze2_trajectory")

plt.show()
