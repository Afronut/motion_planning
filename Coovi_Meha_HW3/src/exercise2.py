
import numpy as np
from math import pi, sin, cos
from numpy import array


class exercise2:
    def __init__(self, start, goal, center, length):
        self.center = center
        self.length = length
        self.goal = goal
        self.start = start
        self.radius = ((2*self.length**2)**.5)/2
        self.epsi = 0.25

    def potential_func(self):
        pot_fun = float('inf')
        pot_att = float('inf')
        scale = .95
        n = 0.01
        iters = 0
        step_size = 0.01
        d_goal_start = 8
        d_q_start = 1
        q = [self.start]
        while np.linalg.norm(pot_fun) > self.epsi:
            pot_repu = 0
            distance_from_obs_edge, edge_point = self.get_dis_to_obs(
                q[iters])
            if np.linalg.norm(array(q[iters])-array(self.goal)) <= scale:
                pot_att = scale * array(q[iters])-array(self.goal)
            else:
                pot_att = d_goal_start * scale * \
                    (array(q[iters]) - array(self.goal)) / \
                    np.linalg.norm(array(q[iters])-array(self.goal))

            for i in range(0, len(self.center)):
                if distance_from_obs_edge[i] <= d_q_start:
                    pot_repu_i = ((n *
                                   (1/d_q_start-1/distance_from_obs_edge[i])) * 1/distance_from_obs_edge[i]**2)*((array(q[i])-array(edge_point[i]))/array(edge_point[i]))
                else:
                    pot_repu_i = 0
                pot_repu += pot_repu_i
            pot_fun = pot_att+pot_repu
            print(np.linalg.norm(pot_fun))
            q.append(q[iters]-step_size*pot_fun)

            iters += 1

    def get_dis_to_obs(self, position):
        num_obs = len(self.center)
        distance_from_obs_edge = []
        edge_point = []
        for i in range(0, num_obs):
            obs_position = self.center[i]
            distance_from_obs_edge.append(np.linalg.norm(
                np.array(obs_position)-np.array(position)))
            diff = array(obs_position)-array(position)
            edge_point_ang = np.angle(complex(diff[0], diff[1]))
            edge_point.append([position[0]-self.radius *
                               np.cos(edge_point_ang), position[1]-self.radius*np.sin(edge_point_ang)])
        return distance_from_obs_edge, edge_point

    def make_obs(self):
        angles = np.linspace(0, 2*pi, 100)
        t_obs = len(self.center)
        ret = []
        # print(angles)
        for i in range(0, t_obs):
            bond_obs = [np.transpose(self.radius*np.cos(angles) + self.center[i][0]),
                        np.transpose(self.radius*np.sin(angles) + self.center[i][1])]
            ret.append(bond_obs)
        return ret

    def compute(self):
        self.potential_func()
