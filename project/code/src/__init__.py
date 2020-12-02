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


def pack_points():
    pack = []
    for elem in const:
        x1, y1, x2, y2 = elem
        pack.append([[x1, y1], [x2, y2]])
    return pack
