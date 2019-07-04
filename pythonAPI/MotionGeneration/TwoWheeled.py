from math import *
from random import random
from .SimulationView import transformation_matrix, SimulationView as sim
import numpy as np

Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.01


def getTranslation(P):
    A = np.matrix([
        [-Kp_rho, 0, 0],
        [0, Kp_rho, -Kp_alpha],
        [0, -Kp_alpha, 0]
    ])
    B= np.matrix([0,-Kp_beta,0]).T
    X = np.matrix([P[0]*cos(P[1]),
                   sin(P[1]),
                   P[1]
                   ]).T
    U = P[2]

    res = A*X + B*U
    return res

def polar_transform(X, G ):
    dX = [g - x for g, x in zip(G, X)]
    rho = sqrt(pow(dX[0],2)+pow(dX[1],2))
    alpha = (atan2(dX[1],dX[0]) - X[2] + pi) % (2 * pi) - pi
    beta = (dX[2]- alpha + pi) % (2 * pi) - pi
    return [rho, alpha, beta]

#transform from vrep frame to robot frame
def transform2robot_frame(pos, point, theta):
    pos = np.asarray(pos)
    point = np.asarray(point)
    T_matrix = np.array([
            [np.sin(theta), np.cos(theta)],
            [np.cos(theta), -1*np.sin(theta)],
            ])
    trans = point-pos
    if trans.ndim >= 2:
        trans = trans.T
        point_t = np.dot(T_matrix, trans).T
    else:
        point_t = np.dot(T_matrix, trans)
    return point_t

def get_distance(points1, points2):
    return np.sqrt(np.sum(np.square(points1 - points2), axis=1))

def body_orientation(rho, alpha):
    return [
        [-cos(alpha), 0],
        [sin(alpha)/rho, -1],
        [-sin(alpha)/rho, 0]
    ]

def state_update(X,U, dt):
    v, w = U
    theta = X[2] + w*dt
    x = X[0] + v*cos(theta)*dt
    y = X[1] + v*sin(theta)*dt
    return [x,y,theta]

def random_state():
    x_start = 20 * random()
    y_start = 20 * random()
    theta_start = 2 * pi * random() - pi
    return [x_start, y_start, theta_start]




if __name__ == '__main__':
    # print(state)
    x_start = 20 * random()
    y_start = 20 * random()
    theta_start = 2 * pi * random() - pi
    X = random_state()
    G = random_state()
    view = sim(X,G,dt)

    print('start ', polar_transform(X, [0] * 3))
    print('goal ', polar_transform(G,[0]*3))

    err = inf
    V = []
    W = []

    while err>0.01:
        P = polar_transform(X, G)
        U = [Kp_rho*P[0], Kp_alpha*P[1]+Kp_beta*P[2]]
        # R = np.matrix(body_orientation(P[0],P[1]))
        # translation = getTranslation(P)
        # R_1 =np.linalg.pinv(R)
        # a = R_1 * translation
        #
        # U = [a[0,0],a[1,0]]
        # print(U, a.T)
        if abs(P[1]) > np.pi / 2:
            # print('negative velocity')
            U[0] *=-1
        v, w = U
        X = state_update(X, U, dt)
        view(X)
        err = P[0]
        V.append(v)
        W.append(w)
    print(f'simulation done minV {min(V)} maxV {max(V)} |  minW {min(W)} maxW {max(W)}')