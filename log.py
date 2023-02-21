import matplotlib.pyplot as plt
import numpy as np

def plot_data():
    with open('log.txt', 'r') as f:
        speed_left = []
        speed_right = []
        input_right = []
        input_left = []
        heading_boat = []
        error = []
        for ligne in f.readlines():
            c1, c2, c3, c4, c5, c6 = ligne.split()
            speed_left.append(c1)
            speed_right.append(c2)
            input_right.append(c3)
            input_left.append(c4)
            heading_boat.append(c5)
            error.append(c6)
        time = np.linspace(0, len(f.readlines()), 10)

    plt.figure("commandes moteur")
    plt.plot(time, speed_left, label="speed left")
    plt.plot(time, speed_right, label="speed_right")
    plt.plot(time, input_right, label="input_left")
    plt.plot(time, input_left, label="input_left")
    plt.figure("heading")
    plt.plot(time, heading_boat)
    plt.legend()
    plt.show()