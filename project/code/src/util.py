from . import obs, const, get_bond
import networkx as nx
from shapely.geometry import Polygon, Point, mapping, LineString
from random import randint, uniform
import numpy as np
import matplotlib.pyplot as plt


class utils:
    def __init__(self, plot_tree, speed=False):
        self.s_id = None
        self.g_id = None
        self.start = None
        self.goal = None
        self.speed = speed
        self.width = 0.4
        self.max_iter, self.step = [100000, 0.5]
        self.obs = obs
        self.bonds = get_bond()
        self.probability, self.eps = [5, 0.25]
        self.paths = None
        self.plot_tree = plot_tree
        self.v = None

    def set_pointions(self, pos):
        self.start, self.goal, self.v = pos

    def set_path(self, path):
        self.paths = path

    def rrt(self):
        pass

    def rrt_goalBias(self):
        G = nx.Graph()
        G.add_node(0, pos=self.start)
        self.s_id = 0
        free_nodes = []
        time_nodes = [[self.start, 0]]
        count = 0
        free_nodes.append(self.start)
        i = 1
        while count < self.max_iter:
            point = self.sample_point(count)
            edge, point, time_node = self.add_edge(
                free_nodes, point, time_nodes)
            is_connectable = False
            if self.speed:
                is_connectable = self.is_timed_free(edge, time_node)
            else:
                is_connectable = self.is_connectable(edge)
            if is_connectable:
                id_t = self.get_node_id(G, edge[1])
                circle = Point(self.goal[0], self.goal[1]).buffer(self.eps)
                p = Point(point[0], point[1])
                if circle.contains(p):
                    self.g_id = i
                    G.add_node(i, pos=self.goal)
                    p1 = Point(self.goal[0], self.goal[1])
                    p2 = Point(edge[1][0], edge[1][1])
                    G.add_edge(i, id_t, )
                    return G, free_nodes, time_nodes
                p1 = Point(edge[0][0], edge[0][1])
                p2 = Point(edge[1][0], edge[1][1])
                G.add_node(i, pos=edge[0])
                G.add_node(id_t, pos=edge[1])
                G.add_edge(i, id_t, weight=p1.distance(p2))
                if self.plot_tree:
                    plt.scatter(point[0], point[1], s=5, c="y")
                    x, y = list(zip(*edge))
                    plt.plot(x, y, c="y")
                    plt.pause(0.001)
                free_nodes.append(point)
                i += 1
                time_nodes.append(time_node)
            count += 1
        return G, free_nodes, time_nodes

    def unfreeze(self, G, free_nodes):
        d = list(G.edges)
        for ed in d:
            for ob in self.obs:
                found = False
                polygon = Polygon(ob)
                d1 = G.nodes[ed[0]]['pos']
                d2 = G.nodes[ed[1]]['pos']
                try:
                    idx1 = free_nodes.index(d1)
                    idx2 = free_nodes.index(d2)
                    for idx in [idx1, idx2]:
                        pn = Point(free_nodes[idx][
                            0], free_nodes[idx][1])
                        c_dis = polygon.boundary.distance(pn)
                        if c_dis <= self.step:
                            G.remove_edge(ed[0], ed[1])
                            free_nodes.pop(idx)
                            # print(c_dis)
                            found = True
                except:
                    pass
                if found:
                    break
        return G, free_nodes

    def get_node_id(self, G, p):
        g_nodes = list(G.nodes.data('pos'))
        for node in g_nodes:
            if node[1] is p:
                return node[0]
        return float('inf')

    def is_done(self, point):
        circle = Point(self.goal[0], self.goal[1]).buffer(self.eps)
        p = Point(point[0], point[1])
        if circle.contains(p):
            return True
        return False

    def sample_point(self, n_iter):
        n_iter = n_iter
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0
        # print(n_iter % (100-self.probability))
        if n_iter % (100-self.probability) != 0:
            min_xy, max_xy = self.bonds
            min_x,  max_x = min_xy
            min_y, max_y = max_xy

        else:
            L = self.eps
            h, k = self.goal
            min_xy, max_xy = [(h-0.5*L, h+0.5*L), (k - 0.5*L, k+0.5*L)]
            min_x,  max_x = min_xy
            min_y, max_y = max_xy
        x = round(uniform(min_x, max_x), 2)
        y = round(uniform(min_y, max_y), 2)

        xy = [x, y]
        return xy

    def too_close(self, edge):
        p1 = Point(edge[0][0], edge[0][1])
        p2 = Point(edge[1][0], edge[1][1])
        dist = p1.distance(p2)
        if dist < self.step-self.step/2:
            return True
        return False

    def is_obstacle_free(self, edge):
        line = LineString(edge).buffer(self.width)
        for tri in self.obs:
            polygon = Polygon(tri)
            if line.intersects(polygon):
                return False
        return True

    def is_colison(self, edge):
        edge_line = LineString(edge).buffer(self.width)
        if not self.paths:
            return True
        for path in self.paths:
            path = [tuple(pt) for pt in path]
            path_line = LineString(path).buffer(self.width)
            if edge_line.intersects(path_line):
                return False
            # print(path)
        return True

    def is_connectable(self, edge):
        if self.is_obstacle_free(edge) and self.is_colison(edge):
            return True
        return False

    def close_obstacle(self, node, point):
        sm = Point(point[0], point[1])
        goal = Point(self.goal[0], self.goal[1])
        dis_sam = goal.distance(sm)
        if dis_sam > self.eps:
            return False
        line_p1_goal = LineString([tuple(node), tuple(point)])
        circle = Point(node[0], node[1]).buffer(self.step)
        inter = circle.intersection(line_p1_goal)
        if inter:
            inter = list(inter.coords)
            inter = list(inter[1])
            inter[0] = round(inter[0], 1)
            inter[1] = round(inter[1], 1)
        else:
            inter = point

        edge = [inter, node]
        if self.is_connectable(edge):
            return False
        return True

    def is_timed_free(self, edge, t_node):
        if not self.is_obstacle_free(edge):
            return False
        t = 0
        t_time = 3*self.step/self.v
        for path in self.paths:
            t = 0
            for i, p in enumerate(path):
                if i == len(path)-1:
                    break
                p1 = Point(p[0], p[1])
                p2 = Point(path[i + 1][0], path[i + 1][1])
                p4 = Point(edge[0][0], edge[0][1])
                line = LineString([p, path[i + 1]]).buffer(self.width)
                dist = p1.distance(p2)
                edge_line = LineString(edge).buffer(self.width)
                if edge_line.intersects(line):
                    inter_point = edge_line.intersection(line)
                    # print(type(inter_point))
                    # print(inter_point)
                    x = []
                    y = []
                    if isinstance(inter_point, (Point, LineString)):
                        x, y = inter_point.coords.xy
                    else:
                        x, y = inter_point.exterior.coords.xy
                    # print(x, y)
                    if not x and not y:
                        continue

                    for i in range(0, len(x)):
                        p3 = Point(x[i], y[i])
                        time_edge_inter = round(
                            t_node[1]-((p4.distance(p3))/self.v), 2)
                        time_path_inter = round(
                            (t+(p1.disjoint(p3)) / self.v), 2)
                        time_diff = abs(time_edge_inter-time_path_inter)
                        if time_diff < t_time:
                            return False
                t += round(dist/self.v, 2)
        return True

    def add_edge(self, nodes, point, time_nodes):
        distances = []
        time_node = []
        for node in nodes:
            p1 = Point(node[0], node[1])
            p2 = Point(point[0], point[1])
            # if not self.close_obstacle(nodes[i], point) and self.is_timed_free(time_nodes[i], point):
            distances.append(p1.distance(p2))
            # else:
            #     distances.append(float("inf"))
        idx = np.argmin(distances)
        node = nodes[idx]
        t_node = time_nodes[idx]
        line = LineString([tuple(node), tuple(point)])
        circle = Point(node[0], node[1]).buffer(self.step)
        inter = circle.intersection(line)
        if inter:
            inter = list(inter.coords)
            inter = list(inter[1])
            inter[0] = round(inter[0], 1)
            inter[1] = round(inter[1], 1)
        else:
            inter = point
        p1 = Point(inter[0], inter[1])
        p2 = Point(node[0], node[1])
        dist = p1.distance(p2)
        edge = [inter, node]
        time_node.append(inter)
        time_node.append(round(dist/self.v+t_node[1], 2))
        return edge, inter, time_node

    def get_rrt_path(self, G=None):
        all_nodes = []
        if not G:
            G, free_nodes, time_nodes = self.rrt_goalBias()
        try:
            path = nx.shortest_path(G, self.s_id, self.g_id, "weight")
        except:
            path = []
        paths = []
        for p in path:
            paths.append(G.nodes[p]["pos"])
        dist = 0
        if paths:
            for i in range(0, len(paths)-1):
                p1 = Point(paths[i][0], paths[i][1])
                p2 = Point(paths[i+1][0], paths[i+1][1])
                d = round(p2.distance(p1), 2)
                dist += d
        x, y = list(zip(*paths))
        node = G.nodes()
        edge = G.edges()
        edges = []
        nodes = []
        time_paths = []
        for n in node:
            ed = G.nodes[n]["pos"]
            nodes.append(ed)
        for e in edge:
            x, y = e
            ed = [G.nodes[x]["pos"], G.nodes[y]["pos"]]
            edges.append(ed)
        return paths, time_nodes

    def prm(self):
        pass

    def gradient_decent(self):
        pass
