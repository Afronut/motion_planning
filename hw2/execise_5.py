
from shapely.geometry import Polygon
from shapely import affinity
from functools import reduce
import operator
import math
import numpy as np


class exercise_5:
    def __init__(self, robot_vect, obs_vect):
        self.robot_vect = robot_vect
        self.obs_vect = obs_vect

    def core(self):
        pass

    def rotate_robot(self, shape, angle=0):
        if not isinstance(shape, Polygon):
            shape = Polygon(shape)
        shape = affinity.rotate(shape, angle, origin="centroid")

        return list(shape.exterior.coords)[:-1]

    def sort_cntclockwise(self, points):
        center = tuple(map(operator.truediv, reduce(
            lambda x, y: map(operator.add, x, y), points), [len(points)] * 2))
        srt = sorted(points, key=lambda coord: (+135 + math.degrees(
            math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
        # srt.reverse();
        srt.append(srt[0])
        return srt

    def cal_angles(self, vecteces):
        degrees = []
        a = np.array(list(vecteces[0]))
        b = np.array([vecteces[0][0]+2, vecteces[0][1]])
        c = np.array(list(vecteces[1]))
        # print([a,b,c])
        ba = c-a
        bc = b-a
        # print(a,b,c)
        cosine_angle = np.dot(ba, bc) / \
            (np.linalg.norm(ba) * np.linalg.norm(bc))

        angle = np.arccos(cosine_angle)
        degrees.append(np.degrees(angle))
        for i in range(1, len(vecteces)-1):
            a = np.array(list(vecteces[i]))
            b = np.array(list(vecteces[i-1]))
            c = np.array(list(vecteces[i+1]))
            ba = c-a
            bc = b-a
        # print(a,b,c)
            cosine_angle = np.dot(ba, bc) / \
                (np.linalg.norm(ba) * np.linalg.norm(bc))
            angle = np.arccos(cosine_angle)
            degrees.append(angle+degrees[-1])
        return degrees

    def construct_polygon(self, vect_obst, vect_robot, degree_ob, degree_robot, degree):
        C_obs = []
        # print(vect_obst)
        # print(vect_robot)
        C_obs.append(vect_obst[0]+vect_robot[0])
        done = False
        i = 0
        j = 0
        ang_obs = degree_ob[0]
        ang_rob = degree_robot[0]
        while not done:
            # print(len(degree_robot))
            # print(len(degree_ob))
            if i == len(vect_robot)-1:
                i = -1
            if j == len(vect_obst)-1:
                    j = -1
            if ang_obs < ang_rob:
                # print(degree_ob)
                # print(degree_robot)
                C_obs.append(vect_robot[i]+vect_obst[j+1])
                if len(degree_ob) != 0:
                    ang_obs = degree_ob.pop(0)
                j += 1
            elif ang_obs > ang_rob:
                # print(degree_ob)
                # print(degree_robot)
                C_obs.append(vect_robot[i+1]+vect_obst[j])
                if len(degree_robot) != 0:
                    ang_rob = degree_robot.pop(0)
                i += 1
            else:
                C_obs.append(vect_robot[i]+vect_obst[j])
                i += 1
                j += 1
                if len(degree_ob) != 0:
                    ang_obs = degree_ob.pop(0)
                if len(degree_robot) != 0:
                    ang_rob = degree_robot.pop(0)
            if (len(degree_robot) == 0 and not len(degree_ob) == 0) or len(C_obs)==(len(vect_robot)+len(vect_obst)-2):
                done = True
            # print(C_obs)
        for i in range(0, len(C_obs)):
            # print(degree)
            C_obs[i] = np.append(np.radians(degree),C_obs[i] )
            C_obs[i] = tuple(C_obs[i])
        return C_obs
