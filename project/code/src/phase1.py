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
            plt.plot([x[i], x[i+1]], [y[i], y[i+1]], lw=4, c="g")


def path():
    fig1 = plt.figure(1)
    pack = pack_points()

    paths = []
    plot_obs()
    for p in pack:
        ut = utils.utils(paths)
        ut.set_pointions(p)
        paths.append(ut.get_rrt_path())
    for path in paths:
        plot_path(path)
    # print(paths)
    plt.show()
