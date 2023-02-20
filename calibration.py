import numpy as np
import numpy.linalg as lng
import time
import sys

sys.path.append("./drivers")
from drivers.imu9_driver_v2 import *
from roblib import rotuv

imu = Imu9IO()
beta = 46 * 10**(-6)
inclination = 64


def calibrate_mag():
    input("Press enter for north calibration")
    xn = imu.read_mag_raw()

    input("Press enter for south calibration")
    xs = imu.read_mag_raw()

    input("Press enter for west calibration")
    xw = imu.read_mag_raw()

    input("Press enter for up calibration")
    xu = imu.read_mag_raw()

    xn = np.array(xn)
    xs = np.array(xs)
    xw = np.array(xw)
    xu = np.array(xu)

    b = -0.5 * (xn + xs)
    X = np.vstack((xn + b, xw + b, xu + b))
    yn = np.array([
        beta * np.cos(inclination * np.pi / 180), 0,
        -beta * np.sin(inclination * np.pi / 180)
    ]).T
    yw = np.array([
        0, -beta * np.cos(inclination * np.pi / 180),
        -beta * np.sin(inclination * np.pi / 180)
    ]).T
    yup = np.array([
        -beta * np.sin(inclination * np.pi / 180), 0,
        beta * np.cos(inclination * np.pi / 180)
    ]).T
    Y = np.vstack((yn, yw, yup))
    A = X @ np.linalg.inv(Y)
    return A, b


def angle_degre(u, v):
    angle_radians = np.arccos(
        (np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))))
    return angle_radians


A, b = calibrate_mag()

while True:
    x1 = imu.read_mag_raw()
    y1 = np.linalg.inv(A) @ (x1 + b)
    y1 = y1 / np.linalg.norm(y1)
    a1 = imu.read_accel_raw()
    a1 = a1 / np.linalg.norm(a1)
    j = np.array([0, 1, 0]).T
    i = np.array([1, 0, 0]).T
    k = np.array([0, 0, 1]).T
    phichap = np.arcsin(angle_degre(a1.T, j))
    thetachap = -np.arcsin(angle_degre(a1.T, i))
    Rh = rotuv(a1, k)
    yh = Rh @ y
    yh1, yh2, yh3 = yh.flatten()
    psichap = -np.arctan2(yh2, yh1)
    print(phichap, thetachap, psichap)
