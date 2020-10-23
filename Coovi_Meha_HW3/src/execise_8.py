from math import cos, sin, radians, acos, degrees
import numpy as np
import matplotlib.pyplot as plt
import time
from src.execise_7 import execise_7
from shapely.geometry import Polygon, Point, mapping, LineString


class execise_8:
    def __init__(self, links=[], n_obs=0, obs_vect=[], max_theta=180, min_theta=0, step=5):
        self.links = links
        self.n_obs = n_obs
        self.obs_vect = obs_vect
        self.max_theta = max_theta
        self.min_theta = min_theta
        self.step = step

    def get_c_space(self):
        c_obstacle_angle = []
        work_space_angle = []
        c_obstacle_ep = []
        work_space_ep = []
        free_space_a = []
        free_space_ep = []
        for i in range(self.min_theta, self.max_theta+self.step, self.step):
            for j in range(self.min_theta, self.max_theta+self.step, self.step):
                links_coordinates = self.rotate_link(self.links, [i, j])
                c_obs_a, c_obs_ep, work_c_a, work_c_ep, f_space_a, f_space_ep = self.collision_check(
                    links_coordinates, (i, j))
                work_space_angle += work_c_a
                work_space_ep += work_c_ep
                if c_obs_a and c_obs_ep:
                    c_obstacle_angle += c_obs_a
                    c_obstacle_ep += c_obs_ep
                else:
                    free_space_a += f_space_a
                    free_space_ep += f_space_ep
        return c_obstacle_angle, c_obstacle_ep, work_space_angle, work_space_ep, free_space_a, free_space_ep

    def plot_obs(self, triangles):
        pass

    def rotate_link(self, cood_links, angles):
        x = 0
        y = 0
        theta = 0
        ret = []
        for i in range(0, len(self.links)):
            theta += angles[i]
            ret.append([x, y])
            x = ret[i][0] + self.links[i]*cos(radians(theta))
            y = ret[i][1] + self.links[i]*sin(radians(theta))
        ret.append([x, y])
        return ret

    def discretise_links(self, points):
        point_discret = []
        for i in range(0, len(points) - 1):
            a = 0
            b = 0
            x_axis = []
            y_axis = []
            num = points[i][1] - points[i + 1][1]
            dem = points[i][0] - points[i + 1][0]
            if abs(dem) < 0.001:
                if (points[i][1] < points[i + 1][1]):
                    x_axis = np.linspace(points[i][1], points[i + 1][1], 500)
                    y_axis = points[i][0] * np.ones(500)
                else:
                    x_axis = np.linspace(points[i+1][1], points[i][1], 500)
                    y_axis = points[i][0] * np.ones(500)
            elif abs(num) < 0:
                if (points[i][0] < points[i + 1][0]):
                    x_axis = np.linspace(points[i][0], points[i + 1][0], 500)
                    y_axis = points[i][0] * np.ones(500)
                else:
                    x_axis = np.linspace(points[i+1][0], points[i][0], 500)
                    y_axis = points[i][0] * np.ones(500)
            else:
                a = (num / dem)
                b = points[i + 1][1]-a*points[i+1][0]
                coeficient = [a, b]  # np.polyfit(points[i], points[i + 1], 1)
                # print(coeficient)
                polynomial = np.poly1d(coeficient)
                x_axis = np.linspace(points[i][0], points[i + 1][0], 500)
                y_axis = polynomial(x_axis)
                d_point = [x_axis, y_axis]
                point_discret.append(d_point)
        return point_discret

    def collision_check(self, points, angle):
        c_obstacle_angle = []
        work_space_angle = []
        c_obstacle_ep = []
        work_space_ep = []
        free_space_a = []
        free_space_ep = []
        anges = tuple(np.around(angle, decimals=4))
        exo7 = execise_7([1, 1], [anges[0], anges[1]])
        cords = exo7.get_coordonates()
        cord = tuple(np.around(cords[-1], decimals=4))
        work_space_angle.append(anges)
        work_space_ep.append(cord)
        for i in range(0, len(points)-1):
            colid = self.is_collision([tuple(points[i]), tuple(points[i+1])])
            if colid:
                c_obstacle_angle.append(anges)
                c_obstacle_ep.append(cord)
            else:
                free_space_a.append(anges)
                free_space_ep.append(cord)
        return c_obstacle_angle, c_obstacle_ep, work_space_angle, work_space_ep, free_space_a, free_space_ep

    def area_triangle(self, a, b, c):
        x1, y1 = a
        x2, y2 = b
        x3, y3 = c
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                    + x3 * (y1 - y2)) / 2.0)

    def area_rectangle(self, a, b, c, d):
        x1, y1 = a
        x2, y2 = b
        x3, y3 = c
        x4, y4 = d
        a = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
        b = ((x2 - x3) ** 2 + (y2 - y3) ** 2) ** .5
        c = ((x3 - x4) ** 2 + (y3 - y4) ** 2) ** .5
        d = ((x4-x1)**2 + (y4-y1)**2)**.5
        return a*b

    def is_collision(self, link):
        # print(self.obs_vect)
        for tri in self.obs_vect:
            polygon = Polygon(tri)
            path = LineString(link)
            c_dis = path.intersects(polygon)
            if c_dis:
                # print("here")
                return True
        return False

    def plot_c_points(self, points):
        pass
