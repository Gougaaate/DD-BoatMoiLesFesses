import numpy as np
from drivers.imu9_driver_v2 import *
from math import *
import time
c = Imu9IO()
# beta = 46*10**(-6)
#
# L = [1, -1, 2, 3]
# i = 0
# while i<4:
#     a = input("vecteur numero :" + str(L[i]))
#     if a == "y" and i == 0:
#         x1 = np.array(c.read_mag_raw())
#         print(x1)
#         print("ok :",L[i])
#     if a == "y" and i == 1:
#         x_1 = np.array(c.read_mag_raw())
#         print(x_1)
#         print("ok :", L[i])
#     if a == "y" and i == 2:
#         x2 = np.array(c.read_mag_raw())
#         print(x2)
#         print("ok :", L[i])
#     if a == "y" and i == 3:
#         x3 = np.array(c.read_mag_raw())
#         print(x3)
#         print("ok :", L[i])
#     i+=1
# b = - (x1.reshape((3,1))+ x_1.reshape((3,1)))/2
# X = np.hstack((x1.reshape((3,1))+b, x2.reshape((3,1))+b, x3.reshape((3,1))+b))
# A = (1/beta)*X
# np.save("A",A)
# np.save("b",b)
#

def current_angle():
    global x1,x_1,x2,x3

    xip = c.read_mag_raw()
    xi = np.array(xip)

    b = np.load("b.npy")
    A = np.load("A.npy")
    y = np.dot(np.linalg.inv(A), xi.reshape((3,1)) + b)

    alpha = (np.arctan2(y[1], y[0])/(2*pi))*360
    return(-alpha)


while True:
    print(current_angle())
    time.sleep(0.5)






#print(y)
#print((alpha)
#print(xi)
#time.sleep(0.5)