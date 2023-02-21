import numpy as np
import numpy.linalg as lng
import time
import sys

sys.path.append("./drivers")
from drivers.imu9_driver_v2 import *

imu = Imu9IO()
beta = 46 * 10**(-6)
inclination = 64


def calibrate_mag():
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

    yn = np.array([[beta * np.cos(degrees_to_radians(inclination))], [0],
                   [-beta * np.sin(degrees_to_radians(inclination))]])

    yw = np.array([[0], [-beta * np.cos(degrees_to_radians(inclination))],
                   [-beta * np.sin(degrees_to_radians(inclination))]])

    yup = np.array([[-beta * np.sin(degrees_to_radians(inclination))], [0],
                    [beta * np.cos(degrees_to_radians(inclination))]])

    Y = np.hstack((yn, yw, yup))
    print(Y.shape)
    A = X @ np.linalg.inv(Y)
    return A, b


def degrees_to_radians(degrees):
    return degrees * np.pi / 180


def radians_to_degrees(radians):
    return radians * 180 / np.pi


def scalarprod(u, v):  # scalar product
    u, v = u.flatten(), v.flatten()
    return sum(u[:] * v[:])


def rotuv(u, v):  # returns rotation with minimal angle such as v=R*u
    # see https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    u = np.array(u).reshape(3, 1)
    v = np.array(v).reshape(3, 1)
    u = (1 / np.linalg.norm(u)) * u
    v = (1 / np.linalg.norm(v)) * v
    c = scalarprod(u, v)
    A = v @ u.T - u @ v.T
    return np.eye(3, 3) + A + (1 / (1 + c)) * A @ A


def get_heading(A, b):
    y = np.linalg.inv(A) @ (np.array(imu.read_mag_raw()).reshape(3, 1) + b)
    return 180 / np.pi * np.arctan2(y[1],
                                    y[0])  # returns boat heading in degrees


def test_heading(A, b):  # calibration test
    L = [[], [], []]
    while True:
        x1 = np.array(imu.read_mag_raw())
        y1 = np.linalg.inv(A) @ (x1 + b)
        y1 = y1 / np.linalg.norm(y1)

        a1 = np.array(imu.read_accel_raw())
        a1 = a1 / np.linalg.norm(a1)
        a1[2] = -a1[2]
        print("a1 = ", a1)

        i = np.array([1, 0, 0]).T
        j = np.array([0, 1, 0]).T
        k = np.array([0, 0, 1]).T

        phi_hat = radians_to_degrees(np.arcsin(scalarprod(a1.T, j)))
        theta_hat = radians_to_degrees(-np.arcsin(scalarprod(a1.T, i)))

        Rh = rotuv(a1, k)
        yh = Rh @ y1
        yh1, yh2, yh3 = yh.flatten()
        psi_hat = radians_to_degrees(-np.arctan2(yh2, yh1))

        L[0].append(phi_hat)
        L[1].append(theta_hat)
        L[2].append(psi_hat)
        alpha, beta, gamma = 0.6, 0.2, 0.2
        if len(L[0]) > 2:
            phi_hat = alpha * L[0][-1] + beta * L[0][-2] + gamma * L[0][-3]
            theta_hat = alpha * L[1][-1] + beta * L[1][-2] + gamma * L[1][-3]
            psi_hat = alpha * L[2][-1] + beta * L[2][-2] + gamma * L[2][-3]

            del L[0][0]
            del L[1][0]
            del L[2][0]

        print("Phi_hat :", phi_hat, "Theta_hat :", theta_hat, "Psi_hat :",
              psi_hat)
        time.sleep(0.2)


if __name__ == "__main__":
    A = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
                  [-5700163.24498797, 80185389.18243794, 884174.92923037],
                  [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
    b = np.array([-1414.5, 1552.5, -4570.5]).T

    test_heading(A, b)
