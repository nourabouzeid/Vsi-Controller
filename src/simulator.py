import numpy as np
import time

class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.pose = np.array([x, y, theta])  
        self.dt = 0.1  

    def update(self, v, w):
        x, y, theta = self.pose
        
        new_x = x + v * np.cos(theta) * self.dt
        new_y = y + v * np.sin(theta) * self.dt
        new_theta = theta + w * self.dt
               
        self.pose = np.array([new_x, new_y, new_theta])
        return self.pose

