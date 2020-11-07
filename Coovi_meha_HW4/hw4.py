from src.exercise_1 import exercise1
from src.exercise_2 import exercise2
from src.exercise_3 import exercise3
import matplotlib.pyplot as plt
import numpy as np
import time
import networkx as nx
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


def test_exo2a(bench=False):
    centers = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    L = 1
    obs = []
    for center in centers:
        h, k = center
        obs.append([(h+0.5*L, k+0.5*L), (h+0.5*L, k - 0.5*L),
                    (h - 0.5*L, k - 0.5*L), (h - 0.5*L, k+0.5*L)])
    fig1 = plt.figure(1)
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
    V, paths, nodes, dist = exo2.compute()
    for node in nodes:
        plt.scatter(node[0], node[1], c="r")
        # plt.pause(0.01)
    for v in V:
        f = list(zip(*v))
        plt.plot(f[0], f[1], c="b", ls='--')
        # plt.pause(0.01)
    if paths:
        x, y = list(zip(*paths))
        for i in range(0, len(x)-1):
            plt.plot([x[i], x[i+1]], [y[i], y[i+1]], lw=4, c="g")
        # plt.pause(0.05)
    plt.scatter([0, 10], [0, 0], color="y")
    plt.annotate("Start", (0, 0), color="y", weight='bold', size=20)
    plt.annotate("goal", (10, 0), color="y", weight='bold', size=20)
    plt.title("n= 200, r=1. path length is : {}".format(round(dist, 2)))

    def benchmark():
        n_r = [(200, 0.5), (200, 1), (200, 1.5), (200, 2),
               (500, 0.5), (500, 1), (500, 1.5), (500, 2)]
        figs = []
        for j in range(0, len(n_r)):
            figt = None
            exo2 = exercise2([start, goal], bonds, n_r[j], obs, 0.2)
            fig = plt.figure(j+2)
            figs.append(fig)
            plt.title("Benchmark for {}".format(n_r[j]))
            dists = []
            t_totals = []
            print(j)
            for i in range(0, 3):
                dd = []
                tt = []
                tic = time.perf_counter()
                g = exo2.sample_workpace(True)
                g, edges = exo2.construct_network_edge(g)
                for k in range(0, 100):
                    V, paths, nodes, dist = exo2.compute(g, edges)
                    if not dist:
                        g = exo2.sample_workpace(True)
                        g, edges = exo2.construct_network_edge(g)
                    dd.append(dist)
                    toc = time.perf_counter()
                    tt.append(toc-tic)
                if dd[0] == 0:
                    print(
                        "No path for option {} run {}. Sampled points too sparse".format(n_r[j], i))
                    # plt.pause(0.001)
                dists.append(dd)
                t_totals.append(tt)
            figs[-1].add_subplot(121)
            plt.title("Path length")
            # for dis in dists:
            plt.boxplot(dists)
            figs[-1].add_subplot(122)
            plt.title("Time taken")
            # for t in t_totals:
            plt.boxplot(t_totals)
            figs.append(figt)
    if(bench):
        benchmark()
    print("Done")
    plt.show()


def test_exo2b(w, bench=False):
    fig1 = plt.figure(1)
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
    bounds = []
    if w == 'w1':
        bounds = [[-1, 13], [-1, 13]]
        start = [0, 0]
        goal = [10, 10]
        w = w1
    elif w == 'w2':
        start = [0, 0]
        goal = [35, 0]
        bounds = [[-6, 36], [-6, 6]]
        w = w2

        w = w2
    dist = 0
    n = 200
    r = 2
    tr = 0
    while not dist:
        plt.close()
        for ob in w:
            xy = list(zip(*ob))
            # print(xy)
            xy[0] = list(xy[0])
            xy[1] = list(xy[1])
            xy[0].append(xy[0][0])
            xy[1].append(xy[1][0])
            plt.fill(xy[0], xy[1], c="r")
        tr += 0.05
        exo2 = exercise2([start, goal], bounds, [n, r], w, tr)
        V, paths, nodes, dist = exo2.compute()

    for node in nodes:
        plt.scatter(node[0], node[1], c="r")
        # plt.pause(0.01)
    for v in V:
        f = list(zip(*v))
        plt.plot(f[0], f[1], c="b", ls='--')
        # plt.pause(0.01)

    if paths:
        x, y = list(zip(*paths))
        for i in range(0, len(x)-1):
            plt.plot([x[i], x[i+1]], [y[i], y[i+1]], lw=4, c="g")
        # plt.pause(0.05)
    plt.scatter([start[0], goal[0]], [start[1], goal[1]], color="y")
    plt.annotate("Start", goal, color="y", weight='bold', size=20)
    plt.annotate("goal", start, color="y", weight='bold', size=20)

    if dist == 0:
        plt.title(
            "No path for option {} run {}. Sampled points too sparse".format(n, r))
    else:
        plt.title("n= 200, r=1. path length is : {}".format(round(dist, 2)))

    def benchmark():
        n_r = [(200, 1), (200, 2), (500, 1), (500, 2), (1000, 1), (1000, 2)]
        figs = []
        for j in range(0, len(n_r)):
            figt = None
            fig = plt.figure(j+2)
            figs.append(fig)
            plt.title("Benchmark for {}".format(n_r[j]))
            dists = []
            t_totals = []
            print(j)
            for i in range(0, 3):
                dd = []
                tt = []
                tic = time.perf_counter()
                tr = 0
                dist = 0
                while not dist:
                    tr += 0.05
                    exo2 = exercise2([start, goal], bonds, n_r[j], w, tr)
                    g = exo2.sample_workpace(True)
                    g, edges = exo2.construct_network_edge(g)
                    for k in range(0, 100):
                        V, paths, nodes, dist = exo2.compute(g, edges)
                        if not dist:
                            k = 0
                            dd.clear()
                            break
                        dd.append(dist)
                        toc = time.perf_counter()
                        tt.append(toc-tic)
                    dists.append(dd)
                t_totals.append(tt)
            figs[-1].add_subplot(121)
            plt.title("Path length")
            # for dis in dists:
            plt.boxplot(dists)
            figs[-1].add_subplot(122)
            plt.title("Time taken")
            # for t in t_totals:
            plt.boxplot(t_totals)
            figs.append(figt)
    if(bench):
        benchmark()
    print("Done")
    plt.show()


def test_exo3(w, bench=False):
    start = [0, 0]
    # start = [-6, 36]
    goal = [10, 10]
    # goal = [35, 0]
    fig1 = plt.figure(1)
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

    bounds = []
    if w == 'w1':
        bounds = [[-1, 13], [-1, 13]]
        start = [0, 0]
        goal = [10, 10]
        w = w1
    elif w == 'w2':
        start = [0, 0]
        goal = [35, 0]
        bounds = [[-6, 36], [-6, 6]]
        w = w2
    for ob in w:
        xy = list(zip(*ob))
        # print(xy)
        xy[0] = list(xy[0])
        xy[1] = list(xy[1])
        xy[0].append(xy[0][0])
        xy[1].append(xy[1][0])
        plt.fill(xy[0], xy[1], c="r")
    n = 5000
    r = 0.5
    p = 5
    eps = 0.25
    exo3 = exercise3([start, goal], bounds, [n, r], w, [p, eps])
    dist, nodes,V, paths = exo3.compute()
    # plt.close()
    # nx.draw(G, nx.get_node_attributes(G, 'pos'),
    #         with_labels=False, node_size=2)
    for node in nodes:
        plt.scatter(node[0], node[1], c="r")
        # plt.pause(0.01)
    for v in V:
        f = list(zip(*v))
        plt.plot(f[0], f[1], c="b", ls='--')
        # plt.pause(0.01)
    plt.title("Path length is {}".format(dist))
    if paths:
        x, y = list(zip(*paths))
        for i in range(0, len(x)-1):
            plt.plot([x[i], x[i+1]], [y[i], y[i+1]], lw=4, c="g")

    def benchmark():
        print("Staring benchmark")
        dists = []
        t_totals = []
        fi2 = plt.figure(3)
        for i in range(0, 3):
            dd = []
            tt = []
            tic = time.perf_counter()
            dist = 0
            exo3 = exercise3([start, goal], bounds, [n, r], w, [p, eps])
            g, nodes = exo3.rrt_goalBias()
            for k in range(0, 100):
                dist, nodes, V, paths = exo3.compute(g)
                dd.append(dist)
                toc = time.perf_counter()
                tt.append(toc-tic)
                dists.append(dd)
            t_totals.append(tt)
        fi2.add_subplot(121)
        plt.title("Path length")
        # for dis in dists:
        plt.boxplot(dists)
        fi2.add_subplot(122)
        plt.title("Time taken (s)")
        # for t in t_totals:
        plt.boxplot(t_totals)
    if(bench):
        benchmark()
    plt.show()


if __name__ == "__main__":
    inp = input("Choose exercise number: For example 1 2 0r 3: ")
    try:
        inp = int(inp)
    except:
        print("Please enter an intger")
        exit(0)

    if inp == 1:
        test_exo1()
    elif inp == 2:
        inp = input("Choose a or b: ")
        if inp == 'a':
            ip = input("Do you wanna run benchmark? yes/no :")
            if ip == 'yes':
                test_exo2a(True)
            else:
                test_exo2a()
        if inp == 'b':
            ip = input("Do you wanna run benchmark? yes/no :")
            w = input("Which workspace? w1 or w2 :")
            if ip == 'yes':
                test_exo2b(w, True)
            else:
                test_exo2b(w)
    elif inp == 3:
        ip = input("Do you wanna run benchmark? yes/no :")
        w = input("Which workspace? w1 or w2 : ")
        if ip == 'yes':
            test_exo3(w, True)
        elif ip == "no":
            test_exo3(w)
