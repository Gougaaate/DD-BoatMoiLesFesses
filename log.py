import matplotlib.pyplot as plt
import plotly.graph_objects as go


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
        pos_x, pos_y = [], []
        for line in p.readlines():
            x, y = line.split()
            pos_x.append(x)
            pos_y.append(y)
    line_a = [0., 0.]
    line_b = [41.87580887, -60.60123502]
    fig = go.Figure()

    fig.add_trace(
        go.Scatter(x=[line_a[0], line_b[0]],
                   y=[line_a[1], line_b[1]],
                   mode='lines+markers',
                   name='goal line'))
    fig.add_trace(
        go.Scatter(x=pos_x,
                   y=pos_y,
                   mode='lines+markers',
                   name='boat trajectory'))

    fig.update_xaxes(title_text="x", title_standoff=25)
    fig.update_yaxes(title_text="y", title_standoff=25)

    fig.update_layout(title_text="Goal line and boat trajectory")

    fig.show()


if __name__ == '__main__':
    # plot_data()
    plot_position()
