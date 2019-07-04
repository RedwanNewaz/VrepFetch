import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from math import pi

gt = np.array(pd.read_csv('history.csv', index_col=0))
test = np.array(pd.read_csv('new_control.csv', index_col=0))

d = 0.331 #wheel axis distance
# d = 0.631 #wheel axis distance
r_w = 0.09751 #wheel radius

def pioneer_robot_model(v_des, omega_des):
    v_r = (v_des+d*omega_des)
    v_l = (v_des-d*omega_des)

    omega_right = v_r/r_w
    omega_left = v_l/r_w
    return omega_right, omega_left
def pioneer_robot_model2(v_des, omega_des):
    v_r = (2*v_des+d*omega_des)
    v_l = (2*v_des-d*omega_des)

    r_ = 2*r_w
    omega_right = v_r/r_
    omega_left = v_l/r_
    return omega_right, omega_left

def diffTouni(v_r, v_l):
    v = (r_w/2)*(v_r+v_l)
    w = (r_w /d) * (v_r - v_l)
    return v,w

def wheel_speed():
    for x,y in zip(v,w):
        a,b = pioneer_robot_model(x,y)
        test_vr.append(a)
        test_vl.append(b)
    plt.plot(vr)
    plt.plot(test_vr)
    plt.title('right wheel velocity')
    plt.legend(['gt', 'test'])
    plt.figure(2)
    plt.plot(vl)
    plt.plot(test_vl)
    plt.title('left wheel velocity')

    plt.legend(['gt','test'])
    plt.show()

def velocity():
    for x,y in zip(vr,vl):
        a,b = diffTouni(x,y)
        test_vr.append(a)
        test_vl.append(b)
    plt.plot(v)
    plt.plot(test_vr)
    plt.title('linear velocity plot')
    plt.legend(['gt', 'test'])
    plt.figure(2)
    plt.plot(w)
    plt.plot(test_vl)
    plt.title('angular velocity plot')
    plt.legend(['gt','test'])
    plt.show()


if __name__ == '__main__':
    # print(gt.shape, test.shape)
    # vr, vl  = gt[:,3], gt[:,4]
    # v, w = test[:, 3], test[:, 4]
    # test_vr, test_vl = [], []
    # velocity()
    # print(vr)
    dtf = pd.read_csv('y_compare.csv',index_col=0)
    dtf.plot()
    # data = np.array(dtf)
    # x = data[:,0]
    # x = (x-pi/2)*2
    # x = np.cumsum(x)
    # plt.plot(x)
    # plt.plot(data[:, 1])
    plt.show()


