import numpy as np
import numpy.linalg as lng
import smbus
import time

from drivers.imu9_driver_v2 import *

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


A, b = calibrate_mag()

while True:
    x = imu.read_mag_raw()
    y = np.linalg.inv(A) @ (x + b)
    y = y / np.linalg.norm(y)
    a1 = imu.read_accel_raw()
    a1 = a1 / np.linalg.norm(a1)
