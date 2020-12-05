from . import util as utils
from . import pack_points, obs, get_bond
import matplotlib.pyplot as plt
import numpy as np
import scipy
from math import pi

i = scipy.pi
dot = scipy.dot
sin = scipy.sin
cos = scipy.cos
ar = scipy.array


def plot_obs():
    # print(obs)
    for ob in obs:
        xy = list(zip(*ob))
        # print(xy)
        xy[0] = list(xy[0])
        xy[1] = list(xy[1])
        xy[0].append(xy[0][0])
        xy[1].append(xy[1][0])
        plt.fill(xy[0], xy[1], c="black")


def plot_path(path):
    if path:
        x, y = list(zip(*path))
        for i in range(0, len(x)-1):
            plt.plot([x[i], x[i+1]], [y[i], y[i+1]], lw=1, c="g")


# def plot_set_points():
#     circle = Point(self.goal[0], self.goal[1]).buffer(self.eps)
#     xy = list(circle.exterior.coords)
#     x, y = list(zip(*xy))
#     plt.fill(x, y, c='m')

#     circle = Point(self.start[0], self.start[1]).buffer(self.eps)
#     xy = list(circle.exterior.coords)
#     x, y = list(zip(*xy))
#     plt.fill(x, y, c='blue')
def get_angle(paths):
    robots = []
    L = 0.4
    for path in paths:
        rots = []
        h, k = path[1]
        robot = [(h+0.5*L, k+0.5*L+0.2), (h+0.5*L, k - 0.5*L-0.2),
                 (h - 0.5 * L, k - 0.5 * L - 0.2), (h - 0.5 * L, k + 0.5 * L + 0.2)]
        rots.append(robot)
        for i in range(0, len(path) - 1):
            h, k = path[i]
            robot = [(h+0.5*L, k+0.5*L+0.2), (h+0.5*L, k - 0.5*L-0.2),
                     (h - 0.5 * L, k - 0.5 * L - 0.2), (h - 0.5 * L, k + 0.5 * L + 0.2)]
            vect1 = np.array(path[i]) - np.array([h, k - 0.5 * L - 0.2])
            vect2 = np.array(path[i])-np.array(path[i+1])
            unit_vector_1 = vect1 / np.linalg.norm(vect1)
            unit_vector_2 = vect2 / np.linalg.norm(vect2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            ang = -np.arccos(dot_product)
            # if ang > pi/4:
            # #     ang = pi/2 - ang
            print(np.degrees(ang))
            rot = dot(ar(robot)-ar(path[i]), ar([[cos(ang), sin(ang)],
                                                 [-sin(ang), cos(ang)]])) + path[i]
            rots.append(rot)
            # plt.scatter(h, (k - 0.5 * L - 0.5), c='r')
            # print(np.degrees(ang))
        h, k = path[-1]
        robot = [(h+0.5*L, k+0.5*L+0.2), (h+0.5*L, k - 0.5*L-0.2),
                 (h - 0.5 * L, k - 0.5 * L - 0.2), (h - 0.5 * L, k + 0.5 * L + 0.2)]
        rots.append(robot)
        robots.append(rots)
    return robots


def plot_motion(motion):

    L = 0.3
    h, k = motion

    robot = [(h+0.5*L, k+0.5*L+0.2), (h+0.5*L, k - 0.5*L-0.2),
             (h - 0.5 * L, k - 0.5 * L - 0.2), (h - 0.5 * L, k + 0.5 * L+0.2)]
    xy = list(zip(*robot))
    # print(xy)
    xy[0] = list(xy[0])
    xy[1] = list(xy[1])
    xy[0].append(xy[0][0])
    xy[1].append(xy[1][0])
    plt.scatter(xy[0], xy[1], c="black",s=5)
    # plt.pause(0.01)
    plt.plot(xy[0][1:3], xy[1][1:3], c="r")


def plot_Rmotion(robot):
    xy = list(zip(*robot))
    # print(xy)
    xy[0] = list(xy[0])
    xy[1] = list(xy[1])
    xy[0].append(xy[0][0])
    xy[1].append(xy[1][0])
    plt.scatter(xy[0], xy[1], c="black")
    plt.plot(xy[0][0:2], xy[1][0:2], c="r")
    plt.pause(0.01)
    # plt.scatter(h, (k - 0.5 * L - 0.5), c='r')


def show_Rmotion(paths):
    fig1 = plt.figure(2)

    cond = []
    robots = get_angle(paths)
    for i in paths:
        cond.append(False)
    count = 0
    while 1:
        plt.clf()
        for i in range(0, len(robots)):
            try:
                plot_path(paths[i])
                plot_Rmotion(robots[i][count])
            except:
                plot_Rmotion(robots[i][-1])
                cond[i] = True
        plt.pause(.1)
        if all(cond):
            break
        count += 1


def show_motion(paths):
    fig1 = plt.figure(1)
    cond = []
    for i in paths:
        cond.append(False)
    count = 0
    while 1:
        plt.clf()
        plot_obs()
        for i in range(0, len(paths)):
            try:
                plot_path(paths[i])
                plot_motion(paths[i][count])
            except:
                plot_motion(paths[i][-1])
                cond[i] = True
        plt.pause(.1)
        if all(cond):
            break
        count += 1


def path():
    pack = pack_points()
    paths = []
    ut = utils.utils(True)
    for p in pack:
        ut.set_path(paths)
        ut.set_pointions(p)
        pa = ut.get_rrt_path()
        if (p[1] in pa):
            print("Path found for :", p)
            paths.append(pa)
    # plot_obs()
    show_motion(paths)
    plt.show()
