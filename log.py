import matplotlib.pyplot as plt


def plot_data():
    with open('log.txt', 'r') as f:
        rpmL = []
        rpmR = []
        goal_rpmL = []
        goal_rpmR = []
        command_pwmL = []
        command_pwmR = []
        heading = []
        error = []
        for line in f.readlines():
            c1, c2, c3, c4, c5, c6, c7, c8 = line.split()
            rpmL.append(float(c1))
            rpmR.append(float(c2))
            goal_rpmL.append(float(c3))
            goal_rpmR.append(float(c4))
            command_pwmL.append(float(c5))
            command_pwmR.append(float(c6))
            heading.append(float(c7))
            error.append(float(c8))
        time = [i for i in range(len(rpmR))]

    plt.figure("rpm")
    plt.plot(time, rpmL, label="rpm left", color="orange")
    plt.plot(time, rpmR, label="rpm right", color="blue")
    plt.plot(time, goal_rpmL, label="goal rpm left", color="red")
    plt.plot(time, goal_rpmR, label="goal rpm right", color="green")
    plt.legend()
    plt.figure("commande moteur")
    plt.plot(time, command_pwmL, label="command rpm left")
    plt.plot(time, command_pwmR, label="command rpm right")
    plt.legend()
    plt.figure("heading")
    plt.plot(time, heading)
    plt.legend()
    plt.figure("error")
    plt.plot(time, error)
    plt.show()


def plot_position():
    with open("position.txt") as p:
        X, Y = [], []
        for line in p.readlines():
            x, y = line.split()
            X.append(x)
            Y.append(y)

        plt.figure("Boat trajectory")
        plt.plot(X, Y, label="trajectory")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    plot_data()
    plot_position()
