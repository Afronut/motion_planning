from random import randint, uniform
from shapely.geometry import Polygon, Point, mapping, LineString
import matplotlib.pyplot as plt
import numpy as np
import math
import networkx as nx


class exercise2(object):
    """
    docstring
    """

    def __init__(self, post, bonds, s_r, obs, treshold=0.2):
        self.start, self.goal = post
        self.samples, self.radius = s_r
        self.s_id = None
        self.g_id = None
        self.obs = obs
        self.bonds = bonds
        self.tresh = treshold
        self.max_iter = 10000

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
        for ob in self.bonds:
            min_x = min(ob)
            min_y = min(ob)
            max_x = max(ob)
            max_y = max(ob)
            MAXs_x.append(max_x)
            MINs_x.append(min_x)
            MAXs_y.append(max_y)
            MINs_y.append(min_y)

        return (min(MINs_x), min(MINs_y)), (max(MAXs_x), max(MAXs_y))

    def sample_workpace(self, nt=False):
        min_xy, max_xy = self.bonds
        min_x,  max_x = min_xy
        min_y, max_y = max_xy
        V = []
        VV = []
        G = nx.Graph()
        if not nt:
            while len(V) <= self.samples:
                x = round(uniform(min_x, max_x), 2)
                y = round(uniform(min_y, max_y), 2)
                xy = [x, y]

                if self.sample_obs_free(x, y) and xy not in V and not self.too_close(V, xy):
                    V.append(xy)
                elif not self.sample_obs_free(x, y):
                    VV.append((xy[:]))
            V.append(self.start)
            V.append(self.goal)
            return V, VV
        if nt:
            i = 0
            while i <= self.samples:
                if i > self.max_iter:
                    print("This is taking too long. Stoping here!")
                    break
                x = round(uniform(min_x, max_x), 2)
                y = round(uniform(min_y, max_y), 2)
                xy = [x, y]
                if self.sample_obs_free(x, y) and xy not in V and not self.too_close(V, xy):
                    V.append(xy)
                    plt.scatter(xy[0], xy[1], c='b')
                    plt.pause(0.005)
                    G.add_node(i, pos=V[-1])
                    i += 1
                elif not self.sample_obs_free(x, y):
                    VV.append((xy[:]))
            self.s_id = i
            i += 1
            self.g_id = i
            G.add_node(self.s_id, pos=self.start)
            G.add_node(self.g_id, pos=self.goal)
            return G

    def sample_obs_free(self, x, y):
        for tri in self.obs:
            polygon = Polygon(tri)
            point = Point(x, y)
            if point.within(polygon):
                return False
        return True

    def too_close(self, nodes, point):
        if not nodes:
            return False
        point = Point(point[0], point[1])
        if isinstance(nodes[0], dict):
            for node in nodes:
                node = list(node.values())[0]
                nd = Point(node[0], node[1])
                dis = point.distance(nd)
                if dis <= self.tresh:
                    return True
        else:
            for node in nodes:
                nd = Point(node[0], node[1])
                dis = point.distance(nd)
                if dis <= self.tresh:
                    return True
        return False

    def construct_edge(self, free_nodes):
        # free_nodes, obs_nodes = self.sample_workpace()
        edges = []

        to_visit = [free_nodes[0][:]]
        visited = []
        # i = 0
        while to_visit:
            node = to_visit.pop()
            visited.append(node)
            circle = Point(node[0], node[1]).buffer(1+0.01)

            for nod in free_nodes:
                point = Point(nod[0], nod[1])
                edge = [node, nod]

                if not circle.contains(point) or nod == node:
                    continue

                elif not self.existed(edges, edge) or self.edge_obs_free(edge):

                    edges.append(edge)
                    if nod not in to_visit and nod not in visited:
                        to_visit.append(nod[:])
        return edges

    def construct_network_edge(self, G):
        edges_dict = []
        edges = []
        data = list(G.nodes.data("pos"))
        pos = data[self.s_id]
        to_visit = [pos]
        visited = []
        while to_visit:
            idx, node = to_visit.pop(0)
            visited.append(node)
            circle = Point(node[0], node[1]).buffer(self.radius+0.01)
            nd = Point(node[0], node[1])
            for d in data:
                idd, nod = d
                point = Point(nod[0], nod[1])
                edge = [node, nod]
                if not circle.contains(point) or nod == node or self.existed(edges, edge):
                    continue

                elif self.edge_obs_free(edge):
                    G.add_edge(idx, idd, weight=point.distance(nd))
                    # for v in V:
                    edges.append(edge)
                    f = list(zip(*edge))
                    plt.plot(f[0], f[1], c="b", ls='--')
                    plt.pause(0.0001)
                    ed = {
                        "pos": [idx, idd],
                        "edge": edge
                    }
                    edges_dict.append(ed)
                    if nod not in to_visit and nod not in visited:
                        to_visit.append(d[:])
        return G, edges_dict

    def smoothing(self):
        pass

    def existed(self, edges, edge):
        if (edge in edges):
            return True
        edge.reverse()
        if (edge in edges):
            return True
        return False

    def edge_obs_free(self, line):
        line = LineString(line)
        for tri in self.obs:
            polygon = Polygon(tri)
            if line.intersects(polygon):
                return False
        return True

    def compute(self, G=None, edges=None):
        nodes = []
        if not G:
            g = self.sample_workpace(True)
            G, edges = self.construct_network_edge(g)
        for ed in edges:
            x, y = ed['pos']
            line = ed['edge']
            if not self.edge_obs_free(line):
                try:
                    g.remove_edge(x, y)
                except:
                    pass
        try:
            path = nx.shortest_path(G, self.s_id, self.g_id, "weight")
        except:
            # print("No path found")
            path = []
        paths = []
        node = G.nodes()
        edge = G.edges()
        edges = []
        # print(edges)
        for n in node:
            ed = G.nodes[n]["pos"]
            nodes.append(ed)
        for e in edge:
            x, y = e
            ed = [G.nodes[x]["pos"], G.nodes[y]["pos"]]
            edges.append(ed)
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
        return edges, paths, nodes, dist
