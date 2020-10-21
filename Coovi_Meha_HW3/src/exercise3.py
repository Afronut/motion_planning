import numoy as np


class exercise_3:
    def __init_(self, start, goal, obs):
        self.obs = obs
        self.start = start
        self.goal = goal

    def meshgrid_obs(self):
        for ob in self.obs:
            ob_cords = list(zip(*ob))
            min_x = min(ob_cords[0])
            min_y = min(ob_cords[1])
            max_x = max(ob_cords[0])
            max_y = max(ob_cords[1])

    def min_max_cords(self):
        MAXs_x = []
        MINs_y = []
        MAXs_y = []
        MINs_x = []
        MAXs_x.append(self.start[0])
        MAXs_y.append(self.start[1])
        MINs_x.append(self.start[0])
        MINs_y.append(self.start[1])

        MAXs_x.append(self.goal[0])
        MAXs_y.append(self.goal[1])
        MINs_x.append(self.goal[0])
        MINs_y.append(self.goal[1])
        for ob in self.obs:
            ob_cords = list(zip(*ob))
            min_x = min(ob_cords[0])
            min_y = min(ob_cords[1])
            max_x = max(ob_cords[0])
            max_y = max(ob_cords[1])
            MAXs_x.append(max_x)
            MINs_x.append(min_x)
            MAXs_y.append(max_y)
            MINs_y.append(min_y)

        return (min(MINs_x), min(MINs_y)), (max(MAXs_x), max(MAXs_y))

    def meshgrid_workspace(self):
        min_c, max_c = self.min_max_cords()
        x = list(range(min_c[0], max_c[0]+1))
        y = list(range(min_c[1], max_c[1]+1))
        X, Y = np.meshgrid(x, y)
        coordinates = []
        for i in range(len(Y)):
            coordinates.append(list(zip(X[i], Y[i])))

        return X, Y, coordinates,

    def make_nodes(self):
        pass
