from src.exercise2 import exercise2
import matplotlib.pyplot as plt
import numpy as np
from math import pi
from mpl_toolkits import mplot3d


def test_exo2_a():
    center = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    length = 1
    exo2 = exercise2(start, goal, center, length)
    obs = exo2.make_obs()
    q, pot_fun, u_atts, u_reps = exo2.compute()
    # print(u_reps[100])
    # print(np.linalg.norm(pot_fun))
    plt.scatter([0, 10], [0, 0])
    plt.xlim([-1, 11])
    plt.ylim([-2, 5])
    plt.plot_
    angles = np.linspace(0, 2*pi, 100)
    bond_obs = [np.transpose(0.25*np.cos(angles) + goal[0]),
                np.transpose(0.25*np.sin(angles) + goal[1])]
    plt.plot(bond_obs[0], bond_obs[1])
    for i in range(0, len(obs)):
        plt.plot(obs[i][0], obs[i][1])
    iters = 10
    for i in range(iters, len(q[0])-iters, iters):
        plt.plot([q[0][i], q[0][i+iters]], [q[1][i], q[1][i+iters]])
        plt.pause(0.01)
    plt.show()


def test_exo2_b():
    center = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    length = 1
    exo2 = exercise2(start, goal, center, length)
    obs = exo2.make_obs()
    q, pot_fun, u_atts, u_reps = exo2.compute()
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    upot = (u_atts+u_reps)
    print(len(q[0]))
    n = int(np.size(upot)/2)
    z = np.reshape(upot, (n, n))

    ax.plot_surface(q[0], q[1], z,
                    cmap='viridis', edgecolor='none')


if __name__ == "__main__":
    test_exo2_b()
