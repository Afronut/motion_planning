import networkx as nx
from shapely.geometry import Polygon, Point, mapping, LineString
from random import randint, uniform
from operator import itemgetter
import numpy as np
import matplotlib.pyplot as plt


class exercise3():
    """
    docstring
    """

    def __init__(self, post, bonds, s_r, obs, p_eps):
        self.s_id = None
        self.g_id = None
        self.start, self.goal = post
        self.max_iter, self.step = s_r
        self.obs = obs
        self.bonds = bonds
        self.probability, self.eps = p_eps

    def rrt_goalBias(self):
        nodes = []
        G = nx.Graph()
        nodes.append(self.start)
        G.add_node(0, pos=self.start)
        self.s_id = 0
        free_nodes = []
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
                        print("here")
                        self.g_id = i
                        G.add_node(i, pos=self.goal)
                        p1 = Point(self.goal[0], self.goal[1])
                        p2 = Point(edge[1][0], edge[1][1])
                        G.add_edge(i, id_t, )
                        return G, free_nodes
                    x, y = list(zip(*edge))
                    plt.plot(x, y, c='b')
                    p1 = Point(edge[0][0], edge[0][1])
                    p2 = Point(edge[1][0], edge[1][1])
                    # print(id_t)
                    G.add_node(i, pos=edge[0])
                    G.add_node(id_t, pos=edge[1])
                    G.add_edge(i, id_t, weight=p1.distance(p2))
                    free_nodes.append(point)

                    plt.scatter(point[0], point[1], c='r')

                    i += 1
            count += 1
        return G, free_nodes

    def unfreeze(self, G, free_nodes):
        d = list(G.edges)
        for ed in d:
            for ob in self.obs:
                found = False
                polygon = Polygon(ob)
                d1 = G.nodes[ed[0]]['pos']
                d2 = G.nodes[ed[1]]['pos']
                # print(free_nodes)
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
                # print(idx1, idx2, ed)
        return G, free_nodes

    def get_node_id(self, G, p):
        g_nodes = list(G.nodes.data('pos'))
        for node in g_nodes:
            if node[1] is p:
                return node[0]
        return float('inf')
        print(g_nodes)

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
            print(n_iter)
            L = self.eps
            h, k = self.goal
            min_xy, max_xy = [(h+0.5*L, k-0.5*L), (h-0.5*L, k + 0.5*L)]
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
        if dist < 0.2:
            return True
        return False

    def is_connectable(self, edge):

        line = LineString(edge)
        for tri in self.obs:
            polygon = Polygon(tri)
            if line.intersects(polygon):
                return False
        return True

    def add_edge(self, nodes, point):
        distances = []
        for node in nodes:
            p1 = Point(node[0], node[1])
            p2 = Point(point[0], point[1])
            distances.append(p1.distance(p2))
        idx = np.argmin(distances)
        node = nodes[idx]
        x, y = list(zip(*[point, node]))
        plt.pause(0.1)
        line = LineString([tuple(node), tuple(point)])
        circle = Point(node[0], node[1]).buffer(self.step)

        inter = circle.intersection(line)
        if inter:
            inter = list(inter.coords)
            inter = list(inter[1])
            inter[0] = round(inter[0], 1)
            inter[1] = round(inter[1], 1)
            # plt.scatter(inter[0], inter[1], c='b')
        else:
            inter = point
        if isinstance(inter[0], tuple):
            pass
        edge = [inter, node]
        return edge, inter

    def compute(self):
        circle = Point(self.goal[0], self.goal[1]).buffer(self.eps)
        xy = list(circle.exterior.coords)
        x, y = list(zip(*xy))
        plt.fill(x, y, c='m')
        plt.scatter(self.start[0], self.start[1], c='g')
        G, nodes = self.rrt_goalBias()
        try:
            path = nx.shortest_path(G, self.s_id, self.g_id, "weight")
        except:
            # print("No path found")
            path = []
        print(path)
        paths = []
        for p in path:
            paths.append(G.nodes[p]["pos"])
        # print(edges)
        dist = 0
        if paths:
            for i in range(0, len(paths)-1):
                p1 = Point(paths[i][0], paths[i][1])
                p2 = Point(paths[i+1][0], paths[i+1][1])
                d = round(p2.distance(p1), 2)
                dist += d
        x,y=list(zip(*paths))
        plt.plot(x,y, lw=2, c='g')
        return G,dist
