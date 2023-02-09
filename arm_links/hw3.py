from src.exercise2 import exercise2
from src.exercise3 import exercise3
from src.exercise4 import exercise4

import matplotlib.pyplot as plt
import numpy as np
from math import pi
from mpl_toolkits import mplot3d


def test_exo2_a():
    center = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    length = 1
    fig = plt.figure(1, figsize=plt.figaspect(2.))
    exo2 = exercise2(start, goal, center, length)
    obs = exo2.make_obs()
    q, pot_fun, path = exo2.compute()

    ax = fig.add_subplot(2, 1, 1)
    ax.scatter([0, 10], [0, 0])
    ax.set_xlim([-1, 11])
    ax.set_ylim([-2, 5])
    angles = np.linspace(0, 2*pi, 100)
    bond_obs = [np.transpose(0.25*np.cos(angles) + goal[0]),
                np.transpose(0.25*np.sin(angles) + goal[1])]
    ax.plot(bond_obs[0], bond_obs[1])
    for i in range(0, len(obs)):
        ax.plot(obs[i][0], obs[i][1])
    iters = 10
    for i in range(iters, len(q[0])-iters, iters):
        ax.plot([q[0][i], q[0][i+iters]], [q[1][i], q[1][i+iters]])
        plt.pause(0.01)
    plt.title("Total path is : {}".format(path))
    # -------------------------------------------------------------- potential fieled
    # ----------------------------------------------------------------------
    x, y, z = exo2.potenfield_generator()
    ax = fig.add_subplot(2, 1, 2, projection='3d')
    X, Y = np.meshgrid(x, y)
    color_map = plt.cm.get_cmap('jet_r')
    ax.plot_surface(X, Y, z,
                    cmap=color_map, edgecolor='none')
    plt.show()


def test_exo2_bw1():
    w1 = [[(1,  1), (2,  1), (2,  5), (1,  5)],
          [(3,  4), (4,  4), (4,  12), (3,  12)],
          [(3,  12), (12,  12), (12,  13), (3,  13)],
          [(12,  5), (13,  5), (13,  13), (12,  13)],
          [(6,  5), (12,  5), (12,  6), (6,  6)]]
    w2 = [
        [(-6,  -6), (25,  -6), (25,  -5), (-6,  -5)],
        [(-6,  5), (30,  5), (30,  6), (-6,  6)],
        [(-6,  -5), (-5,  -5), (-5,  5), (-6,  5)],
        [(4,  -5), (5,  -5), (5,  1), (4,  1)],
        [(9,  0), (10,  0), (10,  5), (9,  5)],
        [(14,  -5), (15,  -5), (15,  1), (14,  1)],
        [(19,  0), (20,  0), (20,  5), (19,  5)],
        [(24,  -5), (25,  -5), (25,  1), (24,  1)],
        [(29,  0), (30,  0), (30,  5), (29,  5)]]
    fig = plt.figure()
    center = w1

    ax = fig.add_subplot(2, 1, 1)
    ax.scatter([0, 10], [0, 10])
    for i in range(0, len(center)):
        x, y = list(zip(*center[i]))
        x = list(x)
        y = list(y)
        x.append(x[0])
        y.append(y[0])
        ax.plot(x, y)
    start = [0, 0]
    goal = [10, 10]
    length = 1

    exo2 = exercise2(start, goal, center, length, True)
    q, pot_fun, path = exo2.compute()
    iters = 10
    for i in range(iters, len(q[0])-iters, iters):
        ax.plot([q[0][i], q[0][i+iters]], [q[1][i], q[1][i+iters]])
        plt.pause(0.01)
    plt.title("Total path is : {}".format(path))

    ret = exo2.dis_to_obs([3, 3])
    # print(ret)

    x, y, z = exo2.potenfield_generator()
    ax = fig.add_subplot(2, 1, 2, projection='3d')
    X, Y = np.meshgrid(x, y)
    color_map = plt.cm.get_cmap('jet_r')
    ax.plot_surface(X, Y, z,
                    cmap=color_map, edgecolor='none')

    plt.show()


def test_exo2_bw2():
    w1 = [[(1,  1), (2,  1), (2,  5), (1,  5)],
          [(3,  4), (4,  4), (4,  12), (3,  12)],
          [(3,  12), (12,  12), (12,  13), (3,  13)],
          [(12,  5), (13,  5), (13,  13), (12,  13)],
          [(6,  5), (12,  5), (12,  6), (6,  6)]]
    w2 = [
        [(-6,  -6), (25,  -6), (25,  -5), (-6,  -5)],
        [(-6,  5), (30,  5), (30,  6), (-6,  6)],
        [(-6,  -5), (-5,  -5), (-5,  5), (-6,  5)],
        [(4,  -5), (5,  -5), (5,  1), (4,  1)],
        [(9,  0), (10,  0), (10,  5), (9,  5)],
        [(14,  -5), (15,  -5), (15,  1), (14,  1)],
        [(19,  0), (20,  0), (20,  5), (19,  5)],
        [(24,  -5), (25,  -5), (25,  1), (24,  1)],
        [(29,  0), (30,  0), (30,  5), (29,  5)]]
    fig = plt.figure()
    center = w2

    ax = fig.add_subplot(2, 1, 1)
    ax.scatter([0, 35], [0, 0])
    for i in range(0, len(center)):
        x, y = list(zip(*center[i]))
        x = list(x)
        y = list(y)
        x.append(x[0])
        y.append(y[0])
        ax.plot(x, y)
    start = [0, 0]
    goal = [35, 0]
    length = 1

    exo2 = exercise2(start, goal, center, length, True)
    # q, pot_fun, path = exo2.compute()
    # iters = 10
    # for i in range(iters, len(q[0])-iters, iters):
    #     ax.plot([q[0][i], q[0][i+iters]], [q[1][i], q[1][i+iters]])
    #     plt.pause(0.01)
    # plt.title("Total path is : {}".format(path))

    # ret = exo2.dis_to_obs([3, 3])
    # print(ret)
    print("This algorith can not find path to goal")
    x, y, z = exo2.potenfield_generator()
    ax = fig.add_subplot(2, 1, 2, projection='3d')
    X, Y = np.meshgrid(x, y)
    color_map = plt.cm.get_cmap('jet_r')
    ax.plot_surface(X, Y, z,
                    cmap=color_map, edgecolor='none')

    plt.show()


def test_exo3_w2():
    w1 = [[(1,  1), (2,  1), (2,  5), (1,  5)],
          [(3,  4), (4,  4), (4,  12), (3,  12)],
          [(3,  12), (12,  12), (12,  13), (3,  13)],
          [(12,  5), (13,  5), (13,  13), (12,  13)],
          [(6,  5), (12,  5), (12,  6), (6,  6)]]
    w2 = [
        [(-6,  -6), (25,  -6), (25,  -5), (-6,  -5)],
        [(-6,  5), (30,  5), (30,  6), (-6,  6)],
        [(-6,  -5), (-5,  -5), (-5,  5), (-6,  5)],
        [(4,  -5), (5,  -5), (5,  1), (4,  1)],
        [(9,  0), (10,  0), (10,  5), (9,  5)],
        [(14,  -5), (15,  -5), (15,  1), (14,  1)],
        [(19,  0), (20,  0), (20,  5), (19,  5)],
        [(24,  -5), (25,  -5), (25,  1), (24,  1)],
        [(29,  0), (30,  0), (30,  5), (29,  5)]]
    start = [0, 0]
    goal = [35, 0]
    exo3 = exercise3(start, goal, w2)
    nodes, path, t_dis = exo3.compute()
    for node in nodes:
        point = node["point"]
        if node["id"] == -1:
            plt.scatter(point[0], point[1], c="r", s=2)
        else:
            plt.scatter(point[0], point[1], c="b", s=2)
        id_t = node["id"]
        label = "{}".format(id_t)
        plt.annotate(label,
                     point,
                     textcoords="offset points",
                     xytext=(0, 2),
                     ha='center')
    for i in range(0, len(w2)):
        x, y = list(zip(*w2[i]))
        x = list(x)
        y = list(y)
        x.append(x[0])
        y.append(y[0])
        plt.plot(x, y)
    plt.plot(path[0], path[1], linewidth=3, c="black")
    plt.title("Total path length is :{} units".format(t_dis))
    plt.show()
    # exo3.meshgrid_obs()


def test_exo3_w1():
    w1 = [[(1,  1), (2,  1), (2,  5), (1,  5)],
          [(3,  4), (4,  4), (4,  12), (3,  12)],
          [(3,  12), (12,  12), (12,  13), (3,  13)],
          [(12,  5), (13,  5), (13,  13), (12,  13)],
          [(6,  5), (12,  5), (12,  6), (6,  6)]]
    w2 = [
        [(-6,  -6), (25,  -6), (25,  -5), (-6,  -5)],
        [(-6,  5), (30,  5), (30,  6), (-6,  6)],
        [(-6,  -5), (-5,  -5), (-5,  5), (-6,  5)],
        [(4,  -5), (5,  -5), (5,  1), (4,  1)],
        [(9,  0), (10,  0), (10,  5), (9,  5)],
        [(14,  -5), (15,  -5), (15,  1), (14,  1)],
        [(19,  0), (20,  0), (20,  5), (19,  5)],
        [(24,  -5), (25,  -5), (25,  1), (24,  1)],
        [(29,  0), (30,  0), (30,  5), (29,  5)]]
    start = [0, 0]
    goal = [10, 10]
    exo3 = exercise3(start, goal, w1)
    nodes, path, t_dis = exo3.compute()
    for node in nodes:
        point = node["point"]
        if node["id"] == -1:
            plt.scatter(point[0], point[1], c="r", s=2)
        else:
            plt.scatter(point[0], point[1], c="b", s=2)
        id_t = node["id"]
        label = "{}".format(id_t)
        plt.annotate(label,
                     point,
                     textcoords="offset points",
                     xytext=(0, 2),
                     ha='center')
    for i in range(0, len(w1)):
        x, y = list(zip(*w1[i]))
        x = list(x)
        y = list(y)
        x.append(x[0])
        y.append(y[0])
        plt.plot(x, y)
    plt.plot(path[0], path[1], linewidth=3, c="black")
    plt.title("Total path length is :{} units".format(t_dis))
    plt.show()
    # exo3.me


def test_exo4(arg):
    exo4 = exercise4(arg)
    exo4.compute()


if __name__ == "__main__":
    ip = input("Please choose exercise number: ex- 3 or 2 :")
    try:
        ip = int(ip)
    except:
        print("Incorrect input")
        exit(0)

    if int(ip) == 2:
        ip = input("Please sub letter: ex- a, bw1 or bw2:")
        if(ip) == "a":
            test_exo2_a()
        elif ip == "bw1":
            test_exo2_bw1()
        elif ip == "bw2":
            test_exo2_bw2()
    if ip == 3:
        ip = input("Please choose workpace: ex- w1,or w2:")
        if(ip) == "w1":
            test_exo3_w1()
        elif ip == "w2":
            test_exo3_w2()
    if ip == 4:
        ip = input("Please sub letter: ex- a, b or b:")
        if(ip) == "a":
            test_exo4("a")
        elif ip == "b":
            test_exo4("b")
        elif ip == "c":
            test_exo4("c")
