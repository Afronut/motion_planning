import numpy as np
import matplotlib.pyplot as plt
from numpy import array


class exercise3:
    def __init__(self, start, goal, obs):
        self.obs = obs
        self.start = start
        self.goal = goal
        self.fig = plt.figure()
        self.step = 1

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
            # x = list(range(min_x, max_x+1, self.step))
            # y = list(range(min_y, max_y+1, self.step))

            x = np.arange(min_x, max_x+1, self.step)
            y = np.arange(min_y, max_y+1, self.step)
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
        x = np.arange(min_c[0]-1, max_c[0]+2, self.step)
        y = np.arange(min_c[1]-1, max_c[1]+2, self.step)
        X, Y = np.meshgrid(x, y)
        coordinates = []
        for xx, yy in zip(X, Y):
            coordinates += (list(zip(xx, yy)))
        return X, Y, coordinates,

    def make_nodes(self):
        next_to_visit = []
        marked = {
            "point": tuple(self.goal),
            "id": 1
        }
        next_to_visit.append(marked)
        has_marked = []
        nodes = []
        nodes.append(marked)
        has_marked.append(marked["point"])
        obs_cords, obs_meshes = self.meshgrid_obs()
        # print(len(obs_cords))
        X, Y, w_cords = self.meshgrid_workspace()
        # print(w_cords)
        w_cords.remove(tuple(self.goal))
        while next_to_visit:
            marking = next_to_visit.pop(0)
            if marking not in nodes:
                nodes.append(marking)
                has_marked.append(marking["point"])
            neighbors = self.get_neighbors(marking["point"])
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
                    if marking["id"] is not -1 and neighbor not in has_marked:
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
        return nodes, has_marked

    def get_neighbors(self, point):
        return [(point[0]-self.step, point[1]),
                (point[0]+self.step, point[1]),
                (point[0], point[1]+self.step),
                (point[0], point[1]-self.step)]

    def make_path(self):
        path = []
        nodes, has_marked = self.make_nodes()
        s_id = has_marked.index(tuple(self.start))
        start = nodes[s_id]
        g_id = has_marked.index(tuple(self.goal))
        goal = nodes[g_id]
        node = start
        t_dis = 0
        path.append(node)
        while nodes is not tuple(self.goal):
            neighbors = self.get_neighbors(node["point"])
            prev_dis = float("inf")
            temp_node = []
            for neighbor in neighbors:
                if neighbor in has_marked:
                    x_id = has_marked.index(neighbor)
                    n_point = nodes[x_id]
                    if n_point["id"] is -1:
                        continue
                    # print(neighbor)
                    dis = np.linalg.norm(array(neighbor)-array(node["point"]))
                    # print(dis)
                    if dis <= prev_dis and n_point["id"] < node["id"]:
                        prev_dis = dis
                        temp_node = n_point
            node = temp_node
            t_dis += prev_dis

            path.append(node)
            if node["id"] == goal["id"]:
                break

        final_path = []
        for p in path:
            final_path.append(p["point"])
        print("Total path length is: ", t_dis)
        final_path = list(zip(*final_path))
        return nodes, final_path, t_dis

    def compute(self):
        return self.make_path()
