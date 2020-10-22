import numpy as np
import matplotlib.pyplot as plt


class exercise3:
    def __init__(self, start, goal, obs):
        self.obs = obs
        self.start = start
        self.goal = goal
        self.fig = plt.figure()

    def meshgrid_obs(self):
        meshes = []
        cords = []
        for ob in self.obs:
            cord = []
            ob_cords = list(zip(*ob))
            min_x = min(ob_cords[0])
            min_y = min(ob_cords[1])
            max_x = max(ob_cords[0])
            max_y = max(ob_cords[1])
            x = list(range(min_x, max_x+1))
            y = list(range(min_y, max_y+1))
            X, Y = np.meshgrid(x, y)
            plt.scatter(X, Y, c="r")
            meshes.append([X, Y])
            for xx, yy in zip(X, Y):
                cord += list(zip(xx, yy))
            cords += cord
        return cords, meshes

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
        x = list(range(min_c[0]-1, max_c[0]+2))
        y = list(range(min_c[1]-1, max_c[1]+2))
        X, Y = np.meshgrid(x, y)
        coordinates = []
        for xx, yy in zip(X, Y):
            coordinates += (list(zip(xx, yy)))
        return X, Y, coordinates,

    def make_nodes(self):
        next_to_visit = []
        marked = {
            "point": self.goal,
            "id": 1
        }
        next_to_visit.append(marked)
        nodes = []
        obs_cords, obs_meshes = self.meshgrid_obs()
        # print(len(obs_cords))
        X, Y, w_cords = self.meshgrid_workspace()
        while next_to_visit:
            marking = next_to_visit.pop(0)
            if marking not in nodes:
                nodes.append(marking)
            # w_cords.remove(marking)
            neighbors = self.get_neighbors(marking["point"])
            print(len(next_to_visit))
            for neighbor in neighbors:
                if neighbor in obs_cords:
                    marked = {
                        "point": neighbor,
                        "id": -1
                    }
                    next_to_visit.append(marked)
                    try:
                        w_cords.remove(neighbor)
                    except:
                        pass
                    try:
                        obs_cords.remove(neighbor)
                    except:
                        pass
                elif neighbor in w_cords:
                    marked = {
                        "point": neighbor,
                        "id": marking["id"]+1
                    }
                    next_to_visit.append(marked)
                    try:
                        w_cords.remove(neighbor)
                    except:
                        pass
                    try:
                        obs_cords.remove(neighbor)
                    except:
                        pass
        return nodes

    def get_neighbors(self, point):
        return [(point[0]-1, point[1]),
                (point[0]+1, point[1]),
                (point[0], point[1]+1),
                (point[0], point[1]-1),
                (point[0]-1, point[1]+1),
                (point[0]+1, point[1]+1),
                (point[0]-1, point[1]-1),
                (point[0]-1, point[1]+1)]

    def compute(self):
        # X, Y, cords = self.meshgrid_workspace()

        # plt.scatter(X, Y)
        # self.meshgrid_obs()
        # plt.show()
        nodes = self.make_nodes()
        for node in nodes:
            point = node["point"]
            if node["id"] == -1:
                plt.scatter(point[0], point[1], c="r")
            else:
                plt.scatter(point[0], point[1], c="b")
        plt.show()
