import numpy as np
import numpy.linalg as lng
import smbus
import time
import sys
sys.path.append("./drivers")
from drivers.imu9_driver_v2 import *

imu = Imu9IO()
beta = 46 * 10**(-6)
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

def angle_degre(u,v):
    angle_radians=np.arccos((np.dot(u, v)/(np.linalg.norm(u)*np.linalg.norm(v))))
    return angle_radians

A,b=generer_b_mat_a()

while True:
    x=imu.read_mag_raw()
    y=np.linalg.inv(A)@(x+b)
    y=y/np.linalg.norm(y)
    a1=imu.read_accel_raw()
    a1=a1/np.linalg.norm(a1)
    j=np.array([0,1,0]).T
    i=np.array([1,0,0]).T
    k=np.array([0,0,1]).T
    phichap=np.arcsin(angle_degre(a1.T,j))
    thetachap=-np.arcsin(angle_degre(a1.T,i))
    Rh=rotuv(a1,k)
    yh=Rh@y
    yh1,yh2,yh3=yh.flatten()
    psichap=-np.arctan2(yh2,yh1)
    print(phichap,thetachap,psichap)
