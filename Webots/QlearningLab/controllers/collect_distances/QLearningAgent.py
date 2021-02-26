import numpy as np

class Agent():
    def __init__(self, lr, gamma, epsilon, episodes, goal):
        self.lr = lr
        self.gamma = gamma
        self.epsilon = epsilon
        # Actions the agent can perform
        self.actions = ["Front", "Light Left", "Light Right", "Left", "Right", "Rotate Left", "Rotate Right", "Back"]
        self.sensors_states = ["Dangerous", "Very close", "Close", "Good", "Far", "Very Far"]
        self.episodes = episodes
        self.goal = goal


        self.QTable = {}
        for i in range(0, width):
            for j in range(0, height):
                for s in range(0,len(self.sensors_states)):
                    for a in self.actions:
                        self.QTable[(i,j)][(s,a)] = 0   # each state will have all actions
                   
