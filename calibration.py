import numpy as np
import sys

sys.path.append("./drivers")

beta = 46 * 10**(-6)
inclination = 64


def DegreesToRadians(degrees):
    return degrees * np.pi / 180


def RadiansToDegrees(radians):
    return radians * 180 / np.pi


def calibrateMag(imu):
    input("Press enter for north calibration")
    xn = np.array(imu.read_mag_raw())

    input("Press enter for south calibration")
    xs = np.array(imu.read_mag_raw())

    input("Press enter for west calibration")
    xw = np.array(imu.read_mag_raw())

    input("Press enter for up calibration")
    xu = np.array(imu.read_mag_raw())

    b = -0.5 * (xn + xs)
    X = np.vstack((xn + b, xw + b, xu + b)).T
    print(X.shape)

    yn = np.array([[beta * np.cos(DegreesToRadians(inclination))], [0],
                   [-beta * np.sin(DegreesToRadians(inclination))]])

    yw = np.array([[0], [-beta * np.cos(DegreesToRadians(inclination))],
                   [-beta * np.sin(DegreesToRadians(inclination))]])

    yup = np.array([[-beta * np.sin(DegreesToRadians(inclination))], [0],
                    [beta * np.cos(DegreesToRadians(inclination))]])

    Y = np.hstack((yn, yw, yup))
    print(Y.shape)
    A = X @ np.linalg.inv(Y)
    return A, b
