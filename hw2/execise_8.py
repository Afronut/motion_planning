from math import cos, sin, radians, acos, degrees
import numpy as np
import matplotlib.pyplot as plt
import time


class execise_8:
    def __init__(self, links=[], n_obs=0, obs_vect=[], max_theta1=360, max_theta2=360, step=2):
        self.links = links
        self.n_obs = n_obs
        self.obs_vect = obs_vect
        self.max_theta1 = max_theta1
        self.max_theta2 = max_theta2
        self.step = step

    def compute(self):
        start = time.clock()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        img = []
        links_coordinate = []
        c_obs = []
        for i in range(0, self.max_theta1, self.step):
            for j in range(0, self.max_theta2, self.step):
                links_coordinates = self.rotate_link(self.links, [i, j])
                discret_points = self.discretise_links(links_coordinates)
                c_obs = self.collision_check(discret_points)

                if c_obs:
                    c_obs = [[i for i, j in c_obs],
                             [j for i, j in c_obs]]
                    theta1 = i * np.ones(len(c_obs[0]))
                    theta2 = np.linspace(0, self.max_theta2, len(c_obs[0]))
                    img = ax.scatter(c_obs[0],  c_obs[1], theta1,
                                     c=theta2, cmap=plt.hot())

                    # plt.pause(0.001)
        fig.colorbar(img)
        end = time.clock()
        plt.title("Program took:{} s".format(end-start))
        plt.show()

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

    def collision_check(self, points):
        c_osctable = []
        for link in points:
            x, y = link
            if (len(x) != len(y)):
                exit()
            for i in range(0, len(x)):
                colid = self.is_collision(x[i], y[i])
                if colid:
                    c_osctable.append((x[i], y[i]))
        return c_osctable

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

    def is_collision(self, x, y):
        # print(self.obs_vect)
        for tri in self.obs_vect:
            # print(tri)
            vect = list(zip(tri[0], tri[1]))
            if (len(vect) == 3):
                # print(vect)
                a, b, c = vect
                area1 = self.area_triangle(a, b, c)
                area2 = self.area_triangle(a, b, (x, y))
                area3 = self.area_triangle(a, (x, y), c)
                area4 = self.area_triangle((x, y), b, c)
                if (area1 == area2 + area3 + area4):
                    return True
            elif (len(vect) == 4):
                vect = list(zip(tri[0], tri[1]))
                a, b, c, d = vect

                if (x > a[0] and x < c[0]) and (y > a[1] and y < c[1]):
                    return True
                elif (x > c[0] and x < a[0]) and (y > c[1] and y < a[1]):
                    return True
        return False

    def plot_c_points(self, points):
        pass
