
import numpy as np
from math import pi, sin, cos
from numpy import array
from shapely import wkt
from shapely.geometry import Polygon, Point, mapping
import matplotlib.pyplot as plt


class exercise2:
    def __init__(self, start, goal, center, length, is_coord=False):
        self.center = center
        self.length = length
        self.goal = goal
        self.start = start
        self.radius = [((2*self.length**2)**.5)/2, ((2*self.length**2)**.5)/2]
        self.epsi = 0.1
        self.is_coord = is_coord

    def potential_func(self):
        # fig = plt.figure()
        pot_fun = float('inf')
        pot_att = float('inf')
        scale = .1
        n = .2
        iters = 0
        step_size = .1
        d_goal_start = 10
        d_q_start = 1
        q = [self.start]
        prev_nor = 0
        prev_q = self.start
        while np.linalg.norm(array(q[iters])-array(self.goal)) > self.epsi:
            if self.is_coord:
                distance_from_obs_edge, edge_point = self.dis_to_obs(
                    q[iters])
            else:
                distance_from_obs_edge, edge_point = self.get_dis_to_obs(
                    q[iters])
            if np.linalg.norm(array(q[iters])-array(self.goal)) <= d_goal_start:
                pot_att = scale * (array(q[iters])-array(self.goal))
            else:
                pot_att = d_goal_start * scale * \
                    (array(q[iters]) - array(self.goal)) / \
                    np.linalg.norm(array(q[iters])-array(self.goal))
            pot_repu_i = 0
            pot_repu = 0
            for i in range(0, len(self.center)):
                if distance_from_obs_edge[i] <= d_q_start:
                    num = 1/d_q_start-1/distance_from_obs_edge[i]
                    grad = (array(q[iters])-array(edge_point[i])) / \
                        array(distance_from_obs_edge[i])
                    d_sq = 1/distance_from_obs_edge[i]**2
                    pot_repu_i = n*num*grad*d_sq

                else:
                    pot_repu_i = 0
                pot_repu += pot_repu_i
            pot_fun = pot_att+pot_repu
            # print(prev_q, q[iters])

            plt.scatter(q[iters][0], q[iters][1])
            plt.pause(0.001)
            # print(q)

            q += [q[iters]-step_size*pot_fun]
            if prev_q[0] == q[iters][0] and prev_q[1] == q[iters][1]:
                ip = input()
                if ip == "u":
                    q[-1][0] = q[-1][0]+step_size/10
                elif ip == "d":
                    q[-1][0] = q[-1][0]-step_size/10

            # print(q)
            prev_q = q[iters]
            iters += 1
        return q, pot_fun

    def get_dis_to_obs(self, position):
        num_obs = len(self.center)
        distance_from_obs_edge = []
        edge_point = []
        for i in range(0, num_obs):
            obs_position = self.center[i]
            distance_from_obs_edge.append(np.linalg.norm(
                np.array(obs_position)-np.array(position)) - self.length)
            diff = array(obs_position)-array(position)
            edge_point_ang = np.angle(complex(diff[0], diff[1]))
            edge_point.append([obs_position[0]-self.radius[i] *
                               np.cos(edge_point_ang), obs_position[1]-self.radius[i]*np.sin(edge_point_ang)])
        return distance_from_obs_edge, edge_point

    def dis_to_obs(self, position):
        num_obs = len(self.center)
        point = Point(position)
        # point = wkt.loads(point)
        distances = []
        points = []
        for i in range(0, num_obs):
            polygon = Polygon(self.center[i])
            # polygon = wkt.loads(polygon)
            c_dis = polygon.boundary.distance(point)
            c_point = polygon.exterior.interpolate(
                polygon.exterior.project(point)).wkt
            c_point = wkt.loads(c_point)
            points.append(list(c_point.coords)[0])
            distances.append(c_dis)
        return distances, points

    def make_obs(self):
        angles = np.linspace(0, 2*pi, 5)
        t_obs = len(self.center)
        ret = []
        for i in range(0, t_obs):
            bond_obs = [np.transpose(self.radius[i]*np.cos(angles) + self.center[i][0]),
                        np.transpose(self.radius[i]*np.sin(angles) + self.center[i][1])]
            ret.append(bond_obs)
        return ret

    def make_centers(self):
        centers = []
        self.radius = []
        for i in range(0, len(self.center)):
            uzip_center = list(zip(*self.center[i]))
            # print(uzip_center)
            center = [sum(uzip_center[0])/len(uzip_center[0]),
                      sum(uzip_center[1])/len(uzip_center[1])]
            radius = np.linalg.norm(array(center)-array(self.center[i][1]))
            self.radius.append(radius)
            centers.append(center)
        self.center = centers

    def compute(self):
        q, pot_fun = self.potential_func()
        path_len = self.path_length(q)
        print("Total Path length is: ", path_len)
        q = list(zip(*q))
        return q, pot_fun, path_len

    def potenfield_generator(self):
        scale = .1
        n = 0.1
        d_goal_start = 1
        d_q_start = 0.2
        n_size = 100
        q1 = np.linspace(-15, 26, n_size)
        q2 = np.linspace(-15, 26, n_size)
        U = np.zeros((n_size, n_size))
        for i in range(0, len(q1)):
            for j in range(0, len(q1)):
                qi = [q1[i], q2[j]]
                if self.is_coord:
                    distance_from_obs_edge, edge_point = self.dis_to_obs(
                        qi)
                else:
                    distance_from_obs_edge, edge_point = self.get_dis_to_obs(
                        qi)
                u_att = 0
                if np.linalg.norm(array(qi)-array(self.goal)) <= d_goal_start:
                    u_att = .5*scale * \
                        np.linalg.norm(array(qi)-array(self.goal))
                else:
                    u_att = d_goal_start*scale * \
                        np.linalg.norm(
                            array(qi)-array(self.goal)) - .5*scale*d_goal_start**2
                rep = 0
                u_rep = 0
                for k in range(0, len(self.center)):
                    if distance_from_obs_edge[k] <= d_q_start:
                        rep = .5*n * \
                            (1/distance_from_obs_edge[k] - 1/d_q_start)**2
                    else:
                        rep = 0
                    u_rep += rep
                u = u_att+u_rep
                if u > 2:
                    u = 2
                U[i][j] = u
        return q1, q2, U

    def path_length(self, q):
        ret = 0
        for i in range(0, len(q)-1):
            ret += np.linalg.norm(q[i+1]-q[i])
        return ret
