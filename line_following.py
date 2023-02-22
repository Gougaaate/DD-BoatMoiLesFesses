import numpy as np

def integration(x, u):
    theta = x[2, 0]
    return np.array([[2 * np.cos(theta)], [2 * np.sin(theta)], [u]])

x = np.array([[-20], [5], [4]])