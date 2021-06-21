import matplotlib.pyplot as plt
import json
import math

font_size = 20

def plotVisitedStates(name):
    f = open(name+".txt", "r")
    visitedStates = json.load(f)
    
    doorNames = json.load(open("./mazes/"+name[-5:]+"_names.txt", "r"))
    x = []
    y = []

    for k in doorNames:
        x.append(doorNames[k])
        if k in visitedStates:
            y.append(visitedStates[k])
        else:
            y.append(0)    

    fig = plt.figure()
    plt.bar(x,y)
    if name == "visited_states_maze1":
        plt.title("Visited states Maze 1", fontsize=font_size)
    else:
        plt.title("Visited states Maze 2", fontsize=font_size)
    plt.xlabel("State", fontsize=font_size)
    plt.ylabel("Number of Visits", fontsize=font_size)
    plt.xticks(fontsize=font_size)
    plt.yticks(fontsize=font_size)
    plt.tight_layout()

def plotAvgLowLvlTimes(name):
    f = open(name+".txt", "r")
    times = []
    for l in f:
        times.append(float(l))

    fig = plt.figure()
    if name == "avgLowLvlTimes_maze1":
        plt.title("Time between High Level States Maze 1", fontsize=font_size)
    else:
        plt.title("Time between High Level States Maze 2", fontsize=font_size)
    
    split = math.floor(len(times)/4)
    times1 = [times[x] for x in range(split)]
    times2 = [None]*len(range(split))
    times2 += [times[x] for x in range(split,split*2)]
    times3 = [None]*len(range(split*2))
    times3 += [times[x] for x in range(split*2,split*3)]
    times4 = [None]*len(range(split*3))
    times4 += [times[x] for x in range(split*3,len(times))]
    plt.plot(times1, marker='o', color="r", label="Starting Position 1")
    plt.plot(times2, marker='o', color="g", label="Starting Position 2")
    plt.plot(times3, marker='o', color="b", label="Starting Position 3")
    plt.plot(times4, marker='o', color="c", label="Starting Position 4")
    plt.ylabel("Time in Low Level (s)", fontsize=font_size)
    plt.xticks(fontsize=font_size)
    plt.yticks(fontsize=font_size)
    plt.legend(fontsize=font_size)
    plt.tight_layout()

def plotSteps(name):
    f = open(name+".txt", "r")
    paths = json.load(f)
    nrSteps = [len(paths[x]) for x in paths]
    fig = plt.figure()
    if name == "steps_maze1":
        plt.title("Steps per episode in Maze 1", fontsize=font_size)
    else:
        plt.title("Steps per episode in Maze 2", fontsize=font_size)

    split = math.floor(len(nrSteps)/4)
    nrSteps1 = [nrSteps[x] for x in range(split)]
    nrSteps2 = [None]*len(range(split))
    nrSteps2 += [nrSteps[x] for x in range(split,split*2)]
    nrSteps3 = [None]*len(range(split*2))
    nrSteps3 += [nrSteps[x] for x in range(split*2,split*3)]
    nrSteps4 = [None]*len(range(split*3))
    nrSteps4 += [nrSteps[x] for x in range(split*3,len(nrSteps))]
    plt.plot(nrSteps1, marker='o', color="r", label="Starting Position 1")
    plt.plot(nrSteps2, marker='o', color="g", label="Starting Position 2")
    plt.plot(nrSteps3, marker='o', color="b", label="Starting Position 3")
    plt.plot(nrSteps4, marker='o', color="c", label="Starting Position 4")
    plt.xlabel("Number of episodes", fontsize=font_size)
    plt.ylabel("Number of steps", fontsize=font_size)
    plt.xticks(fontsize=font_size)
    plt.yticks(fontsize=font_size)
    plt.legend(fontsize=font_size)
    plt.tight_layout()

def plotRewardSum(name):
    f = open(name+".txt", "r")
    rwds = []
    for l in f:
        rwds.append(float(l))
    fig = plt.figure()
    if name == "rwdSum_maze1":
        plt.title("Reward Sum in Maze 1", fontsize=font_size)
    else:
        plt.title("Reward Sum in Maze 2", fontsize=font_size)

    split = math.floor(len(rwds)/4)
    rwds1 = [rwds[x] for x in range(split)]
    rwds2 = [None]*len(range(split))
    rwds2 += [rwds[x] for x in range(split,split*2)]
    rwds3 = [None]*len(range(split*2))
    rwds3 += [rwds[x] for x in range(split*2,split*3)]
    rwds4 = [None]*len(range(split*3))
    rwds4 += [rwds[x] for x in range(split*3,len(rwds))]
    plt.plot(rwds1, marker='o', color="r", label="Starting Position 1")
    plt.plot(rwds2, marker='o', color="g", label="Starting Position 2")
    plt.plot(rwds3, marker='o', color="b", label="Starting Position 3")
    plt.plot(rwds4, marker='o', color="c", label="Starting Position 4")
    plt.xlabel("Number of episodes", fontsize=font_size)
    plt.ylabel("Reward Sum", fontsize=font_size)
    plt.xticks(fontsize=font_size)
    plt.yticks(fontsize=font_size)
    plt.legend(fontsize=font_size)
    plt.tight_layout()
    
plotVisitedStates("visited_states_maze1")
plotAvgLowLvlTimes("avgLowLvlTimes_maze1")
plotSteps("steps_maze1")
plotRewardSum("rwdSum_maze1")

plotVisitedStates("visited_states_maze2")
plotAvgLowLvlTimes("avgLowLvlTimes_maze2")
plotSteps("steps_maze2")
plotRewardSum("rwdSum_maze2")

plt.show()
