# simple Grid World problem
import tkinter as tk
import numpy as np
from PIL import Image, ImageTk
import time
import io
import sys
import matplotlib.pyplot as plt

class Game():
    def __init__(self):
        self.title = input("Graphs Window Title: ")
        # Ask user for grid size
        self.r, self.col = [int(x) for x in input("Grid Size (rows,columns): ").replace("(","").replace(")","").split(",")]
        self.width = 50*self.col
        self.height = 50*self.r
        
        self.root = tk.Tk()
        # Create canvas (top left corner is (0,0) bottom right corner is (height,widht))
        self.canvas = tk.Canvas(self.root, height=self.height, width=self.width, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # List of cells not occupied
        self.availablePos = []
        for x in range(self.col):
            for y in range(self.r):
                self.availablePos.append([x*50,y*50])

        # Initial Agent Position (Bottom Left Corner)
        self.initial_pos = [0,50*(self.r-1)]
        self.agent_pos = self.initial_pos.copy()

        # make corner positions unavailable (used for full map exploration)
        self.availablePos.remove(self.initial_pos)
        self.availablePos.remove([0,0])
        self.availablePos.remove([50*(self.col-1),0])
        self.availablePos.remove([50*(self.col-1),50*(self.r-1)])


        self.loadImages()
        self.drawGrid()
        self.drawAgent()

        print("Use Left Mouse Button to create obstacles\nUse Right Mouse button to select Goal Position\nTo run the agent hit <Return>!")
        self.winPos = []
        self.obs = []
        # Left mouse button to draw obstacles
        self.drawObs = self.root.bind("<ButtonPress-1>", self.drawObstacles)
        # Right mouse button to draw goal
        self.dGoal = self.root.bind("<ButtonPress-3>", self.drawGoal)

    def loadImages(self):
        # https://stackoverflow.com/questions/43009527/how-to-insert-an-image-in-a-canvas-item      HOW TO ADD IMAGES TO CANVAS
        # https://www.codegrepper.com/code-examples/delphi/how+to+resize+image+in+python+tkinter    HOW TO RESIZE IMAGES IN TKINTER
        self.vic = ImageTk.PhotoImage(Image.open("images/vic.png").resize((50,50), Image.ANTIALIAS))
        self.agentIm = ImageTk.PhotoImage(Image.open("images/robot.png").resize((50,50), Image.ANTIALIAS))
        self.leftIm = ImageTk.PhotoImage(Image.open("images/left.png").resize((50,50), Image.ANTIALIAS))
        self.rightIm = ImageTk.PhotoImage(Image.open("images/right.png").resize((50,50), Image.ANTIALIAS))
        self.upIm = ImageTk.PhotoImage(Image.open("images/up.png").resize((50,50), Image.ANTIALIAS))
        self.downIm = ImageTk.PhotoImage(Image.open("images/down.png").resize((50,50), Image.ANTIALIAS))

    def drawAgent(self):
        self.agent = self.canvas.create_image(self.agent_pos[0], self.agent_pos[1], image=self.agentIm, anchor='nw')

    def drawGrid(self):
        # Create vertical lines at intevals of 50
        for i in range(0, self.width, 50):
            self.canvas.create_line([(i, 0), (i, self.height)], tag='grid_line')
        # Create horizontal lines at intevals of 50
        for i in range(0, self.height, 50):
            self.canvas.create_line([(0, i), (self.width, i)], tag='grid_line')

    def drawGoal(self, event):
        topX = event.x//50*50
        topY = event.y//50*50
        if len(self.winPos) == 0:   
            self.canvas.create_image(topX,topY, image=self.vic, anchor='nw', tags="goalPos")
            self.winPos.append(topX)
            self.winPos.append(topY)
        else:
            if [topX, topY] in self.availablePos:
                self.availablePos.append([int(x) for x in self.canvas.coords("goalPos")])
                self.canvas.delete("goalPos")
                self.canvas.create_image(topX,topY, image=self.vic, anchor='nw', tags="goalPos")
                self.winPos.clear()
                self.winPos.append(topX)
                self.winPos.append(topY)
        self.availablePos.remove(self.winPos)

    def drawObstacles(self, event):        
        topX = event.x//50*50
        topY = event.y//50*50
        if [topX, topY] in self.availablePos:
            self.obs.append([topX,topY])
            self.canvas.create_rectangle(topX, topY, topX+50, topY+50, fill="#000000")
            self.availablePos.remove([topX, topY])

    def drawResult(self, Qvalues):
        self.canvas.delete(self.agent)
        for p in Qvalues:
            if p != tuple(self.winPos) and [p[0], p[1]] not in self.obs:    
                k = max(Qvalues[p], key=Qvalues[p].get)
                if k == 'up':
                    self.canvas.create_image(p[0], p[1], image=self.upIm, anchor='nw')
                elif k == 'down':
                    self.canvas.create_image(p[0], p[1], image=self.downIm, anchor='nw')
                elif k == 'left':
                    self.canvas.create_image(p[0], p[1], image=self.leftIm, anchor='nw')
                elif k == 'right':
                    self.canvas.create_image(p[0], p[1], image=self.rightIm, anchor='nw')

    # Functions to animate the robot
    def left(self, event):
        tmp = [self.agent_pos[0]-50, self.agent_pos[1]]
        if self.agent_pos[0] == 0 or tmp in self.obs:    # hit wall or middle obstacle
            pass
        else:
            time.sleep(0.05)
            self.agent_pos[0]-=50
            self.canvas.move(self.agent, -50, 0)
            self.canvas.update()

    def right(self, event):
        tmp = [self.agent_pos[0]+50, self.agent_pos[1]]
        if self.agent_pos[0] == 50*(self.col-1) or tmp in self.obs:
            pass
        else:
            time.sleep(0.05)
            self.agent_pos[0]+=50
            self.canvas.move(self.agent, 50, 0)
            self.canvas.update()
            
    def up(self, event):
        tmp = [self.agent_pos[0], self.agent_pos[1]-50]
        if self.agent_pos[1] == 0 or tmp in self.obs:
            pass
        else:
            time.sleep(0.05)
            self.agent_pos[1]-=50
            self.canvas.move(self.agent, 0, -50)
            self.canvas.update()

    def down(self, event):
        tmp = [self.agent_pos[0], self.agent_pos[1]+50]
        if self.agent_pos[1] == 50*(self.r-1) or tmp in self.obs:
            pass
        else:
            time.sleep(0.05)
            self.agent_pos[1]+=50
            self.canvas.move(self.agent, 0, 50)
            self.canvas.update()

    # check if the agent reached the goal   
    def gameEnd(self, pos):
        return pos == self.winPos

    def reset(self):
        self.canvas.move(self.agent, self.initial_pos[0]-self.agent_pos[0], self.initial_pos[1]-self.agent_pos[1])
        self.canvas.update()
        self.agent_pos = self.initial_pos.copy()

    def setAgentPos(self, pos):     
        self.initial_pos[0] = pos[0]*50
        self.initial_pos[1] = pos[1]*50


class Agent():
    def __init__(self):
        self.game = Game()
        self.goal = self.game.winPos
        self.lr = 0.1       # alpha
        self.gamma = 0.95    # discount factor
        self.epsilon = 0.1  # e-greedy
        self.episodes = 500 # nr of runs
        self.actions = ['up', 'down', 'left', 'right']

        # list of steps taken in a episode
        self.states = []

        # reward sum
        self.rewardSum = [0]*self.episodes

        # list of steps from last episode
        # self.lastStates = []

        # dict with nr of visits each cell has
        self.visitedCells = dict()

        # dict with all paths taken
        self.paths = dict()
        
        # Initialize Q-table
        self.Qvalues = {}
        for i in range(0, self.game.height, 50):         # height/50 = number of rows
            for j in range(0, self.game.width, 50):      
                self.Qvalues[(j,i)] = {}
                self.visitedCells[(j,i)] = 0
                for a in self.actions:
                    self.Qvalues[(j,i)][a] = 0   # Each state has all 4 actions

        # start the agent when Return is pressed
        self.game.root.bind("<Return>", self.run)
        self.game.root.mainloop()

    def reward(self, pos, old_pos=None):
        if pos == self.goal:
            return 50       # win reward
        elif tuple(pos) == old_pos:
            return -50      # agent hit a wall(not drawn on canvas) or obstacle
        else:
            return -10        # living penalty

    # choose the best action (the one with highest q_value) or a random one (exploitation and exploration)
    # https://towardsdatascience.com/simple-reinforcement-learning-q-learning-fcddc4b6fe56
    def chooseAction(self):
        action = ''

        if np.random.uniform(0,1) < self.epsilon:
            action = np.random.choice(self.actions)
        else:
            # https://thispointer.com/python-how-to-get-all-keys-with-maximum-value-in-a-dictionary/
            # Select the action with highest q_value (if more than one with the same max q_value then select a random one from those)
            maxAction = max(self.Qvalues[tuple(self.game.agent_pos)].items(), key=lambda x: x[1])
            listofmaxActions = [k for k, v in self.Qvalues[tuple(self.game.agent_pos)].items() if v == maxAction[1]]
            action = np.random.choice(listofmaxActions)

        return action

    # get max Q_value of all the 4 actions in the next state
    def maxQ(self, agentPos):
        highest_q = 0
        for a in self.actions:
            nxt_q = self.Qvalues[agentPos][a]
            if nxt_q >= highest_q:
                highest_q = nxt_q
        return highest_q

    # updates coordinates and do the robot animation
    def takeAction(self, action):
        if action == 'up':
            self.game.up(None)
        elif action == 'down':
            self.game.down(None)
        elif action == 'left':
            self.game.left(None)
        else:
            self.game.right(None)
 
    def reset(self):
        time.sleep(0.1)
        self.game.reset()
        self.states.clear()
        time.sleep(0.1)

    def run(self,event):
        # block mouse clicks after simulation begins
        self.game.root.unbind("<ButtonPress-1>", self.game.drawObs)
        self.game.root.unbind("<ButtonPress-2>", self.game.dGoal)
        r = 0   # episode count

        while r < self.episodes:
            if self.game.gameEnd(self.game.agent_pos):
                self.visitedCells[tuple(self.game.agent_pos)] += 1
                self.paths[r] = self.states.copy()
                final_rwd = self.reward(self.game.agent_pos)
                self.rewardSum[r] += final_rwd

                # all actions in the Goal cell have the same reward
                for a in self.actions:
                    self.Qvalues[tuple(self.game.agent_pos)][a] = final_rwd
                
                # Change start position to better explor the map
                if r == self.episodes/4-1:
                    self.epsilon = 0.1
                    self.game.setAgentPos([0,0])	# top left
                if r == self.episodes/2-1:
                    self.epsilon = 0.1
                    self.game.setAgentPos([self.game.col-1,0])	#top right
                if r == 3*self.episodes/4-1:
                    self.epsilon = 0.1
                    self.game.setAgentPos([self.game.col-1,self.game.r-1])	# bottom right

                if r == self.episodes/4-11:
                    self.epsilon = 0
                elif r == self.episodes/2-11:
                    self.epsilon = 0
                elif r == 3*self.episodes/4-11:
                    self.epsilon = 0
                elif r == self.episodes-11:
                    self.epsilon = 0

                r += 1
                if r < self.episodes:
                    self.reset()
                print(r, self.epsilon)
            else:
                action = self.chooseAction()
                # save each agent move
                self.states.append([tuple(self.game.agent_pos), action])
                # count how many times the agent passes in each cell
                self.visitedCells[tuple(self.game.agent_pos)] += 1

                # Update Q-Table
                cur_pos = tuple(self.game.agent_pos)
                cur_q = self.Qvalues[cur_pos][action]
                self.takeAction(action)
                rwd = self.reward(self.game.agent_pos, cur_pos)
                self.rewardSum[r] += rwd
                new_q = cur_q + self.lr * (rwd + self.gamma * self.maxQ(tuple(self.game.agent_pos)) - cur_q)
                self.Qvalues[cur_pos][action] = round(new_q, 3)

        self.game.drawResult(self.Qvalues)
        self.printAndSaveTables()
        self.drawGraphs()

    def printAndSaveTables(self):
        print("--------------Final Q-Table--------------\n")
        try:
            with open("Qtable.txt", "a") as f1:
                f1.write("\n--------NEW RUN--------")
                f1.write("\nFormat: (rows,columns)")
                f1.write("\nGRID Size (" + str(self.game.r) + ", " + str(self.game.col) + 
                    ")\nGoal Position: (" + str(self.goal[0]//50+1) + ", " + str(self.goal[1]//50+1) +")\nNumber of episodes: " + str(self.episodes))
                f1.write("\n--------------Final Q-Table--------------\n")
                for i in self.Qvalues:
                    s = "Pos: (" + str(i[0]//50+1) + ", " + str(i[1]//50+1) + ") Actions Values: " + str(self.Qvalues[i]) + " Nr Visits: " + str(self.visitedCells[i]) + "\n"
                    print(s)
                    f1.write(s)
                f1.close() 
        except:
            f1 = open("Qtable.txt", "w+")
            f1.write("\nGRID Size (" + str(self.game.r) + ", " + str(self.game.col) + 
                ")\nGoal Position: (" + str(self.goal[0]//50+1) + ", " + str(self.goal[1]//50+1) +")\nNumber of episodes: " + str(self.episodes))
            f1.write("--------------Final Q-Table--------------\n")
            for i in self.Qvalues:
                s = "Pos: (" + str(i[0]//50+1) + ", " + str(i[1]//50+1) + ") Actions Values: " + str(self.Qvalues[i]) + " Nr Visits: " + str(self.visitedCells[i]) +  "\n"
                print(s)
                f1.write(s)
            f1.close()

        try:
            with open("Paths.txt", "a") as f2:
                f2.write("\n--------NEW RUN--------\n")
                for p in self.paths:
                    path = ""
                    for s in self.paths[p]:
                        path += s[1] + " "
                    f2.write("Round: " + str(p+1) + "\nPath: " + path + "\n\n")
                f2.close()
        except:        
            f2 = open("Paths.txt", "w+")
            for p in self.paths:
                path = ""
                for s in self.paths[p]:
                    path += s[1] + " "
                f2.write("Round: " + str(p+1) + "\nPath: " + path + "\n\n")
            f2.close()

    def drawGraphs(self):
        # Convert paths dictionary to list of lists
        nrSteps = [len(self.paths[x]) for x in self.paths]
        # Create matrix to use as heatmap
        mat = np.zeros((self.game.r, self.game.col))

        for x in self.visitedCells:
            mat[x[1]//50,x[0]//50] = self.visitedCells[x]

        # https://stackoverflow.com/questions/7744697/how-to-show-two-figures-using-matplotlib/47956978
        fig, (plt1, plt3) = plt.subplots(2)
        fig.canvas.set_window_title(self.game.title)
        plt1.plot(nrSteps, marker='o', linestyle="solid")
        plt1.set_title("Steps per episode")
        plt1.set(xlabel="Number of episodes", ylabel="Number of steps")        
        plt3.plot(self.rewardSum)
        plt3.set(xlabel="Number of episodes", ylabel="Reward Sum")
        
        fig2, plt2 = plt.subplots(1)
        fig2.canvas.set_window_title(self.game.title)
        plt2.matshow(mat)
        # https://matplotlib.org/3.1.1/gallery/images_contours_and_fields/image_annotated_heatmap.html
        # Loop over data dimensions and create text annotations.
        for i in range(self.game.r):
            for j in range(self.game.col):
                text = plt2.text(j, i, mat[i, j],
                               ha="center", va="center", color="black")
        plt2.set_title("Grid HeatMap")
        plt2.set(xlabel="Coordinate X", ylabel="Coordinate Y")
        # https://stackoverflow.com/questions/14406214/moving-x-axis-to-the-top-of-a-plot-in-matplotlib
        plt2.xaxis.set_label_position("top")

        plt.show()
        
agent = Agent()