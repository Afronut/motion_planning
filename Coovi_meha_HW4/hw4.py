from src.exercise_1 import exercise1
from src.exercise_2 import exercise2
import matplotlib.pyplot as plt
import numpy as np
# from math import round


def test_exo1():
    start = (0, 'start')
    end = (0, 'end')
    v = [start, (3, 'a'), (2, 'b'), (3, 'c'), (3, 'd'), (1, 'e'), (3, 'f'),
         (2, 'g'), (1, 'h'), (2, 'i'), (3, 'j'), (2, 'k'), (3, 'l'), end]
    e = [('start', 'a'), ('start', 'b'), ('start', 'c'), ('a', 'd'), ('a', 'e'), ('a', 'f'), ('b', 'g'), ('b', 'h'),
         ('b', 'i'), ('c', 'j'), ('c', 'k'), ('c', 'l'), ('g', 'end'), ('i', 'end'), ('k', 'end'), ('e', 'end')]
    w = [1, 1, 1, 1, 1, 3, 4, 1, 2, 1, 1, 1, 3, 3, 2, 3]
    exo1 = exercise1(start, end, [v, e, w])
    path, iter_num, queues = exo1.a_star_path()
    print("A start path is: {} with {} total iterations".format(path, iter_num))
    path, iter_num, queues = exo1.dijkstra_path()
    print("dijkstra path is: {} with {} total iterations".format(path, iter_num))


def test_exo2():
    centers = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    L = 1
    obs = []
    for center in centers:
        h, k = center
        obs.append([(h+0.5*L, k+0.5*L), (h+0.5*L, k - 0.5*L),
                    (h - 0.5*L, k - 0.5*L), (h - 0.5*L, k+0.5*L)])
    fig = plt.figure()
    plt.scatter([0, 10], [0, 0])
    for ob in obs:
        xy = list(zip(*ob))
        # print(xy)
        xy[0] = list(xy[0])
        xy[1] = list(xy[1])
        xy[0].append(xy[0][0])
        xy[1].append(xy[1][0])
        plt.fill(xy[0], xy[1], c="r")
    bonds = [[-1, 11], [-3, 3]]
    exo2 = exercise2([start, goal], bonds, [200, 1], obs)
    V, paths, nodes= exo2.compute()
    # print(V)
    # V = list(zip(*V))
    # VV = list(zip(*VV))
    for node in nodes:
        plt.scatter(node[0], node[1], c="b")
        plt.pause(0.01)
    # for v in V:
    #     f = list(zip(*v))
    #     plt.plot(f[0], f[1], c="b", ls='--')
    #     plt.pause(0.01)
    x, y = list(zip(*paths))
    plt.plot(x, y, lw=4, c="g")
    print("Done")
    plt.show()


if __name__ == "__main__":
    test_exo2()
