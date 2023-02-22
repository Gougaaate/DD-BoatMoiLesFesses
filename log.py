import numpy as np
import matplotlib.pyplot as plt


def plot_data():
    with open('log.txt', 'r') as f:
        speed_left = []
        speed_right = []
        input_left = []
        input_right = []
        heading_boat = []
        error = []
        for ligne in f.readlines():
            c1, c2, c3, c4, c5, c6 = ligne.split()
            speed_left.append(float(c1))
            speed_right.append(float(c2))
            input_left.append(float(c3))
            input_right.append(float(c4))
            heading_boat.append(float(c5))
            error.append(float(c6))
        time = [i for i in range(len(speed_right))]

    plt.figure("vitesse moteur")
    plt.plot(time, speed_left, label="speed left")
    plt.plot(time, speed_right, label="speed_right")
    plt.legend()
    plt.figure("commande moteur")
    plt.plot(time, input_left, label="input_left")
    plt.plot(time, input_right, label="input_right")
    plt.legend()
    plt.figure("heading")
    plt.plot(time, heading_boat)
    plt.legend()
    plt.figure("erreur")
    plt.plot(time, error)
    plt.show()

if __name__ == '__main__':
    plot_data()