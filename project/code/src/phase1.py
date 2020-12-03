from . import util as utils
from . import pack_points, obs
import matplotlib.pyplot as plt


def plot_obs():
    # print(obs)
    for ob in obs:
        xy = list(zip(*ob))
        # print(xy)
        xy[0] = list(xy[0])
        xy[1] = list(xy[1])
        xy[0].append(xy[0][0])
        xy[1].append(xy[1][0])
        plt.fill(xy[0], xy[1], c="r")


def plot_path(path):
    if path:
        x, y = list(zip(*path))
        for i in range(0, len(x)-1):
            plt.plot([x[i], x[i+1]], [y[i], y[i+1]], lw=1, c="g")


def plot_motion(motion):

    L = 0.4
    h, k = motion
    robot = [(h+0.5*L, k+0.5*L), (h+0.5*L, k - 0.5*L),
             (h - 0.5 * L, k - 0.5 * L), (h - 0.5 * L, k + 0.5 * L)]
    xy = list(zip(*robot))
    # print(xy)
    xy[0] = list(xy[0])
    xy[1] = list(xy[1])
    xy[0].append(xy[0][0])
    xy[1].append(xy[1][0])
    plt.fill(xy[0], xy[1], c="r")


def show_motion(paths):
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
    fig1 = plt.figure(1)
    pack = pack_points()
    paths = []
    # plot_obs()
    for p in pack:
        ut = utils.utils(paths)
        ut.set_pointions(p)
        pa = ut.get_rrt_path()
        if (p[1] in pa):
            paths.append(pa)
    show_motion(paths)
    # print(paths)
    plt.show()
