import pandas as dp
import os
import sys
import numpy as np

constraints = os.path.join(sys.path[0], "src/constraints.csv")
obstacles = os.path.join(sys.path[0], "src/obstacles.csv")
const_def = dp.read_csv(constraints)
obs_def = dp.read_csv(obstacles)

const = [list(row) for row in const_def.values]
obs = [list(row) for row in obs_def.values]
obstacles = []

for ob in obs:
    b = [tuple(row) for row in np.reshape(ob, (-1, 2))]
    obstacles.append(b)
    obs = obstacles
# print(obs)


def pack_points():
    pack = []
    for elem in const:
        x1, y1, x2, y2, v = elem
        pack.append([[x1, y1], [x2, y2], v])
    return pack


def get_bond():
    X = []
    Y = []
    for ob in obs:
        x, y = list(zip(*ob))
        X += list(x)
        Y += list(y)
    pack = []
    for elem in const:
        x1, y1, x2, y2, v = elem
        pack.append([[x1, y1], [x2, y2]])
    for elem in pack:
        x, y= list(zip(*elem))
        # print(x)
        X += list(x)
        Y += list(y)
    minx = np.min(X)-5
    miny = np.min(Y)-5
    maxx = np.max(X)+5
    maxy = np.max(Y)+5
    b = [[minx, maxx], [miny, maxy]]
    return b
