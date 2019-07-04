import matplotlib.pyplot as plt
from math import *
import numpy as np

def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

class SimulationView(object):
    x_traj, y_traj = [], []
    def __init__(self, start, goal, dt):
        self.__start = start
        self.__goal = goal
        self.__dt = dt

    def show_background(self):
        plt.cla()
        x_start, y_start, theta_start = self.__start
        x_goal, y_goal, theta_goal = self.__goal
        plt.arrow(x_start, y_start, cos(theta_start),
                  sin(theta_start), color='r', width=0.1)
        plt.arrow(x_goal, y_goal, cos(theta_goal),
                  sin(theta_goal), color='g', width=0.1)
        # plt.plot(self.__pathx,self.__pathy, 'k')

    def add_path(self,path):
        self.__pathx, self.__pathy = [], []
        for p in path:
            if len(p)==3:
                x, y, theta = p
            else:
                x, y = p
            self.__pathx.append(x)
            self.__pathy.append(y)


    def plot_vehicle(self, x, y, theta):  # pragma: no cover
        # Corners of triangular vehicle when pointing to the right (0 radians)
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        T = transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        self.x_traj.append(x)
        self.y_traj.append(y)

        plt.plot(self.x_traj, self.y_traj, 'b--')

        plt.xlim(-5, 5)
        plt.ylim(-5, 5)

        plt.pause(self.__dt)

    def __call__(self, robot):
        x, y, theta = robot
        # self.show_background()
        self.plot_vehicle(x, y, theta)