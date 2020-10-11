import numpy as np
import matplotlib.pyplot as plt
from execise_5 import exercise_5
from execise_7 import execise_7
import geopandas as gpd
from matplotlib import animation
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def exercise5():
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
        ax.set_ylim([-4, 4])
        ax.set_xlim([-4, 6])
        ax.set_zlim([-3, 2])
        plt.pause(0.05)
        angle += 1

    # anim = animation.ArtistAnimation(fig, img, interval=0.05)
    plt.show()


def execise7(vect, degrees, end_pt):
    exo7 = execise_7(vect, degrees, end_pt)
    cord, degre = exo7.compute()
    x, y = np.array(cord).T
    plt.figure()
    plt.plot(x, y, lw=5)
    plt.scatter(x, y, marker="o", lw=7)
    plt.plot([x[0], x[0]+1], [y[0], y[0]], ls="--", c="r")
    plt.plot([x[1], x[1]+1], [y[1], y[1]], ls="--", c="r")
    plt.plot([x[2], x[2]+1], [y[2], y[2]], ls="--", c="r")
    plt.plot([x[3], x[3]+1], [y[3], y[3]], ls="--", c="r")
    plt.text(x[0]+.5, y[0], "{}".format(degre[0]))
    plt.text(x[1]+.5, y[1], "{}".format(round(degre[1] + degre[0], 2)))
    plt.text(x[2]+.5, y[2], "{}".format(round(degre[2] + degre[1] + degre[0], 2)))
    plt.text(x[3]+.5, y[3], "{}".format(round(degre[2] + degre[1] + degre[0], 2)))
    plt.scatter(0, 0, c='r', marker="o", lw=7)
    # plt.arrow(x[3], y[3], .01, .01, head_width=0.5)
    # plt.xlim(-0.1)
    # plt.ylim(-0.1)
    plt.text(-1, 0, "Root")
    plt.title("Arms positions. Angles are shown with respect to the horizontal")
    plt.axis("off")
    plt.show()
# exercise5()


if __name__ == "__main__":
    execise7([10, 10, 5], [], [1, 7, 90])
