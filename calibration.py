import numpy as np
import numpy.linalg as lng
import smbus
import time
from roblib import *

from drivers.imu9_driver_v2 import *

imu = Imu9IO()
beta = 46 * 10 ** (-6)
I = 64


def generer_b_mat_a():
    print("mettre le bateau vers le nord")
    time.sleep(5.0)
    xn = imu.read_mag_raw()

    print("mettre le bateau vers le sud")
    time.sleep(5.0)
    xs = imu.read_mag_raw()

    print("mettre le bateau vers l'ouest")
    time.sleep(5.0)
    xw = imu.read_mag_raw()

    print("mettre le bateau vers l'up")
    time.sleep(5.0)
    xu = imu.read_mag_raw()
    b = -0.5*(xn+xs)
    X = np.vstack((xn+b,xw+b,xu+b))
    yn = np.array([beta*np.cos(64*np.pi/180),0,beta*np.sin(64*np.pi/180)]).T
    yw = np.array([0, -beta * np.cos(64 * np.pi / 180),-beta*np.sin(64*np.pi/180)]).T
    yup = np.array([-beta*np.sin(64*np.pi/180),0,beta*np.cos(64*np.pi/180)]).T
    Y = np.vstack((yn,yw,yup))
    A=X@np.linalg.inv(Y)
    return A,b

A,b=generer_b_mat_a()

while True:
    x=imu.read_mag_raw()
    y=np.linalg.inv(A)@(x+b)
    y=y/np.linalg.norm(y)
    a1=imu.read_accel_raw()
    a1=a1/np.linalg.norm(a1)
    phichap=np.arcsin()


