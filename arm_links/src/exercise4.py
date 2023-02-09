from src.execise_7 import execise_7
from src.execise_8 import execise_8
import matplotlib.pyplot as plt
import numpy as np
from numpy import array


class exercise4:
    def __init__(self, arg):
        self.arg = arg
        self.obs = []
        self.goal_ = [-2.0, 0]
        self.start_ = [2.0, 0]
        self.goal = []
        self.start = []
        self.step = 5
        if arg == "a":
            self.obs = [[(0.25, 0.25), (0, 0.75), (-0.25, 0.25)]]
        elif arg == "b":
            self.obs = [[(-0.25, 1.1), (-0.25, 2), (0.25, 2), (0.25, 1.1)],
                        [(-2, -2), (-2, -1.8), (2, -1.8), (2, -2)]]
        elif arg == "c":
            self.obs = [[(-0.25, 1.1), (-0.25, 2), (0.25, 2), (0.25, 1.1)],
                        [(-2, -0.5), (-2, -0.3), (2, -0.3), (2, -0.5)]]

    def compute(self):
        exo8 = []
        if self.arg == "a" or self.arg == "b":
            exo8 = execise_8([1, 1], len(self.obs), self.obs, 180, -180)
        if self.arg == "c":
            exo8 = execise_8([1, 1], len(self.obs), self.obs, 360, 0)
        c_space_a, c_space_ep, workspace_a, workspace_ep, f_a, f_ep = exo8.get_c_space()
        # print(len(workspace_a))
        self.goal = f_a[f_ep.index(tuple(self.goal_))]
        self.start = f_a[f_ep.index(tuple(self.start_))]
        nodes, path = self.make_path(workspace_a, c_space_a)
        # print(nodes)
        c_space_a, c_space_ep, workspace_a, workspace_ep, f_a, f_ep = exo8.get_c_space()

        # print(self.goal, self.start)
        # print(path)
        X, Y = list(zip(*c_space_a))
        plt.figure(1)
        plt.scatter(X, Y)
        plt.scatter(self.goal[0], self.goal[1])
        plt.scatter(self.start[0], self.start[1])
        plt.plot(path[0], path[1], c="r", lw=3)

        cords = self.get_path_pos(list(zip(path[0], path[1])))
        # print(cords)
        for j in range(0, len(cords), 5):
            plt.figure(j+2)
            # print(cords[j])
            for i in range(0, len(self.obs)):
                x, y = list(zip(*self.obs[i]))
                x = list(x)
                y = list(y)
                x.append(x[0])
                y.append(y[0])
                plt.plot(x, y)
            x, y = list(zip(*cords[j]))
            plt.scatter(cords[j][0][0], cords[j][0][1], c="g")
            plt.scatter(cords[j][-1][0], cords[j][-1][1], c="r")
            plt.plot(x, y)
            plt.xlim([-5, 5])
            plt.ylim([-5, 5])
        plt.show()

    def make_nodes(self, w_cords, obs_cords):
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
        w_cords.remove(tuple(self.goal))
        while len(next_to_visit) != 0:
            marking = next_to_visit.pop(0)
            if marking not in nodes:
                nodes.append(marking)
                has_marked.append(marking["point"])
            neighbors = self.get_neighbors(marking["point"])
            # print(neighbors)
            for neighbor in neighbors:
                # if neighbor[0] >= 0 and neighbor[1] >= 0:
                #     print(neighbors)
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
                    # print(neighbor)
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
                    elif marking["id"] is -1 and neighbor not in has_marked:
                        next_to_visit.append(marked)
                        try:
                            w_cords.remove(neighbor)
                        except:
                            pass
                        try:
                            obs_cords.remove(neighbor)
                        except:
                            pass

        # print(len(has_marked))
        return nodes, has_marked

    def get_neighbors(self, point):
        return [(point[0]-self.step, point[1]),
                (point[0]+self.step, point[1]),
                (point[0], point[1]+self.step),
                (point[0], point[1]-self.step)]

    def make_path(self,  w_cords, obs_cords):
        path = []
        nodes, has_marked = self.make_nodes(w_cords, obs_cords)
        # print(has_marked)
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
                    if n_point["id"] < node["id"] and dis < prev_dis:
                        prev_dis = dis
                        temp_node = n_point
            node = temp_node
            t_dis += prev_dis
            # print(t_dis)

            path.append(node)
            # print(node, goal)
            if node["id"] == goal["id"]:
                break

        final_path = []
        for p in path:
            final_path.append(p["point"])
        # print(start, s_id)
        final_path = list(zip(*final_path))
        return nodes, final_path

    def get_path_pos(self, path):
        ret = []
        # print(path)
        for point in path:
            exo7 = execise_7([1, 1], point)
            cords = exo7.get_coordonates()
            ret.append(cords)
        return ret
