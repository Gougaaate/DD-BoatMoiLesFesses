import numpy as np
import sys
from get_current_heading import degToRad
from drivers.imu9_driver_v2 import Imu9IO

sys.path.append("./drivers")

beta = 46 * 10**(-6)
inclination = 64

imu = Imu9IO()


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

    yn = np.array([[beta * np.cos(degToRad(inclination))], [0],
                   [-beta * np.sin(degToRad(inclination))]])

    yw = np.array([[0], [-beta * np.cos(degToRad(inclination))],
                   [-beta * np.sin(degToRad(inclination))]])

    yup = np.array([[-beta * np.sin(degToRad(inclination))], [0],
                    [beta * np.cos(degToRad(inclination))]])

    Y = np.hstack((yn, yw, yup))
    print(Y.shape)
    A = X @ np.linalg.inv(Y)
    return A, b


if __name__ == '__main__':
    A, b = calibrateMag(imu)
    print("A: ", A)
    print("b: ", b)
