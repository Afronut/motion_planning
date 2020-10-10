import numpy as np
import matplotlib.pyplot as plt
from execise_5 import exercise_5
import geopandas as gpd
from matplotlib import animation
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

if __name__ == "__main__":
    angle = 0
    step = 1
    fig = plt.figure()
    obstacle = np.array([[0, 0], [0, 2], [1, 2]])
    robot = -obstacle
    exo5 = exercise_5(robot, obstacle)
    ax = Axes3D(fig)
    while(angle < 360):
        sorted_obstacle = exo5.sort_cntclockwise(obstacle)
        degrees_obstacle = exo5.cal_angles(sorted_obstacle)
        # print(degrees_obstacle)
        robot_rot = exo5.rotate_robot(robot, angle)
        sorted_robot = exo5.sort_cntclockwise(robot_rot)
        degrees_robot = exo5.cal_angles(sorted_robot)
        # print(degrees_robot)
        C_obs = exo5.construct_polygon(
            sorted_obstacle, sorted_robot, degrees_obstacle, degrees_robot, angle)
        # t3 = plt.Polygon(C_obs)
        verts = [list(C_obs)]
        # print(verts)

        ax.add_collection3d(Poly3DCollection(verts))
        ax.set_ylim([-4, 7])
        ax.set_xlim([-4, 7])
        ax.set_zlim([-4, 7])
        plt.pause(0.05)
        angle += 5

    # anim = animation.ArtistAnimation(fig, img, interval=0.05)
    plt.show()

    # plt.figure()
    # plt, show()
