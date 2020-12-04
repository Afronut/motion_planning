from . import obs, const
import networkx as nx
from shapely.geometry import Polygon, Point, mapping, LineString
from random import randint, uniform
import numpy as np
import matplotlib.pyplot as plt


class utils:
    def __init__(self, paths):
        self.s_id = None
        self.g_id = None
        self.start = None
        self.goal = None
        self.width = 0.4
        self.max_iter, self.step = [100000, 0.2]
        self.obs = obs
        self.bonds = [[-1, 13], [-1, 13]]
        self.probability, self.eps = [5, 0.25]
        self.paths = paths

    def set_pointions(self, pos):
        self.start, self.goal = pos

    def rrt(self):
        pass

    def rrt_goalBias(self):
        nodes = []
        G = nx.Graph()
        nodes.append(self.start)
        G.add_node(0, pos=self.start)
        self.s_id = 0
        free_nodes = []
        all_nodes = []
        count = 0
        free_nodes.append(self.start)
        i = 1
        while count < self.max_iter:
            point = self.sample_point(count)
            nodes.append(point)
            edge, point = self.add_edge(free_nodes, point)
            if self.is_connectable(edge):
                if self.too_close(edge) and i > 5:
                    # G, free_nodes=self.unfreeze(G, free_nodes)
                    pass
                else:
                    id_t = self.get_node_id(G, edge[1])
                    circle = Point(self.goal[0], self.goal[1]).buffer(self.eps)
                    p = Point(point[0], point[1])
                    if circle.contains(p):
                        self.g_id = i
                        G.add_node(i, pos=self.goal)
                        p1 = Point(self.goal[0], self.goal[1])
                        p2 = Point(edge[1][0], edge[1][1])
                        G.add_edge(i, id_t, )
                        return G, free_nodes, all_nodes
                    p1 = Point(edge[0][0], edge[0][1])
                    p2 = Point(edge[1][0], edge[1][1])
                    G.add_node(i, pos=edge[0])
                    G.add_node(id_t, pos=edge[1])
                    G.add_edge(i, id_t, weight=p1.distance(p2))
                    free_nodes.append(point)
                    i += 1
                all_nodes.append(point)
            count += 1
        return G, free_nodes, all_nodes

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
        if n_iter % (100-self.probability):
            min_xy, max_xy = self.bonds
            min_x,  max_x = min_xy
            min_y, max_y = max_xy

        else:
            L = self.eps
            h, k = self.goal
            min_xy, max_xy = [(h+0.5*L, h-0.5*L), (k + 0.5*L, k-0.5*L)]
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
        if dist < 0.05:
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

    def add_edge(self, nodes, point):
        distances = []
        for node in nodes:
            p1 = Point(node[0], node[1])
            p2 = Point(point[0], point[1])
            distances.append(p1.distance(p2))
        idx = np.argmin(distances)
        node = nodes[idx]
        x, y = list(zip(*[point, node]))
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
        if isinstance(inter[0], tuple):
            pass
        edge = [inter, node]
        return edge, inter

    def get_rrt_path(self, G=None):
        all_nodes = []
        if not G:
            G, free_nodes, all_nodes = self.rrt_goalBias()
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
        for n in node:
            ed = G.nodes[n]["pos"]
            nodes.append(ed)
        for e in edge:
            x, y = e
            ed = [G.nodes[x]["pos"], G.nodes[y]["pos"]]
            edges.append(ed)
        return paths



    def prm(self):
        pass

    def gradient_decent(self):
        pass
