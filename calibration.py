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
    X = np.vstack((xn + b, xw + b, xu + b))
    yn = np.array([
        beta * np.cos(degrees_to_radians(inclination)), 0,
        -beta * np.sin(degrees_to_radians(inclination))
    ]).T
    yw = np.array([
        0, -beta * np.cos(degrees_to_radians(inclination)),
        -beta * np.sin(degrees_to_radians(inclination))
    ]).T
    yup = np.array([
        -beta * np.sin(degrees_to_radians(inclination)), 0,
        beta * np.cos(degrees_to_radians(inclination))
    ]).T
    Y = np.vstack((yn, yw, yup))
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


def get_angle(A, b):
    y = np.linalg.inv(A) @ (np.array(imu.read_mag_raw()).reshape(3, 1) + b)
    return 180 / np.pi * np.arctan2(y[1],
                                    y[0])  # returns boat heading in degrees


def test(A, b):  # to test calibration
    while True:
        print(get_angle(A, b))


A, b = calibrate_mag()
print("A :", A)
print("b :", b)

#A = np.array([[-1.02678427e+08, 2.32084459e+07, -1.84551143e+07],
              #[-1.65787253e+08, 5.66821659e+07, -7.27692951e+07],
              #[-6.11925317e+07, 3.97221478e+07, -1.10497139e+08]])
#b = np.array([[-455.5], [1160.], [-5808.5]])

# while True:
#     x1 = np.array(imu.read_mag_raw())
#     y1 = np.linalg.inv(A) @ (x1 + b)
#     y1 = y1 / np.linalg.norm(y1)
#     a1 = np.array(imu.read_accel_raw())
#     a1 = a1 / np.linalg.norm(a1)
#     i = np.array([1, 0, 0]).T
#     j = np.array([0, 1, 0]).T
#     k = np.array([0, 0, 1]).T
#     phi_hat = radians_to_degrees(np.arcsin(scalarprod(a1.T, j)))
#     theta_hat = radians_to_degrees(-np.arcsin(scalarprod(a1.T, i)))
#     Rh = rotuv(a1, k)
#     yh = Rh @ y1
#     yh1, yh2, yh3 = yh.flatten()
#     psi_hat = radians_to_degrees(-np.arctan2(yh2, yh1))
#     print("Phi_hat :", phi_hat, "Theta_hat :", theta_hat, "Psi_hat :", psi_hat)
#     time.sleep(0.2)

while True:
    print(get_angle(A, b))
    time.sleep(0.2)
