import numpy as np
import time
import sys

sys.path.append("./drivers")

beta = 46 * 10**(-6)
inclination = 64


def DegreesToRadians(degrees):
    return degrees * np.pi / 180


def RadiansToDegrees(radians):
    return radians * 180 / np.pi


def scalarProd(u, v):  # scalar product
    u, v = u.flatten(), v.flatten()
    return sum(u[:] * v[:])


def rotuv(u, v):  # returns rotation with minimal angle such as v=R*u
    u = np.array(u).reshape(3, 1)
    v = np.array(v).reshape(3, 1)
    u = (1 / np.linalg.norm(u)) * u
    v = (1 / np.linalg.norm(v)) * v
    c = scalarProd(u, v)
    A = v @ u.T - u @ v.T
    return np.eye(3, 3) + A + (1 / (1 + c)) * A @ A


def getHeadingSimple(imu, A, b):
    y = np.linalg.inv(A) @ (np.array(imu.read_mag_raw()) + b)
    return 180 / np.pi * np.arctan2(y[1],
                                    y[0])  # returns boat heading in degrees


def getEulerAngles(imu, A, b):  # calibration test
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

        phi_hat = RadiansToDegrees(np.arcsin(scalarProd(a1.T, j)))
        theta_hat = RadiansToDegrees(-np.arcsin(scalarProd(a1.T, i)))

        Rh = rotuv(a1, k)
        yh = Rh @ y1
        yh1, yh2, yh3 = yh.flatten()
        psi_hat = RadiansToDegrees(-np.arctan2(yh2, yh1))

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
