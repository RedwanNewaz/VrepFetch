from vrepper import SimulatorConnection, simx_opmode_oneshot
from math import *
from time import sleep
from random import random
from MotionGeneration import sim
from MotionGeneration import state_update
from collections import defaultdict
import pandas as pd
import ctypes

Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.015 # make sure simulation dt 200 ms



def polar_transform(X, G ):
    dX = [g - x for g, x in zip(G, X)]
    rho = sqrt(pow(dX[0],2)+pow(dX[1],2))
    alpha = (atan2(dX[1],dX[0]) - X[2] + pi) % (2 * pi) - pi
    beta = (dX[2]- alpha + pi) % (2 * pi) - pi
    return [rho, alpha, beta]


class RobotModel(SimulatorConnection):
    def __init__(self):
        super().__init__()
        self.robotHandle = self.venv.get_object_by_name('fetch')
        self.lad = self.venv.get_object_by_name('look_ahead')
        p = self.state()
        self.lad.set_position(p[0],p[1]+0.5,0.05)

    def state(self):
        p = self.robotHandle.get_position()
        q = self.robotHandle.get_orientation()
        return [p[0],p[1],q[1]]
    def __repr__(self):
        p = self.state()
        return '[{:.3f}, {:.3f}, {:.3f}]'.format(p[0],p[1],p[2])

    def SendData(self, u):
        packedData = self.robotHandle.env.simxPackFloats(u)
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)
        returnCode = self.robotHandle.env.simxWriteStringStream("input", raw_bytes, simx_opmode_oneshot)

    def __call__(self, G):
        sleep(10)
        self.lad.set_position(G[0], G[1], 0.05)


if __name__ == '__main__':

    G = [6,3,0.05]
    robot = RobotModel()
    try:
        robot.start()
        robot(G)
        input('press any numeric key to quit ')
        robot.stop()
    except:
        robot.stop()


            # robot.stop()