import numpy as np
import matplotlib.pyplot as plt
from execise_5 import execise_5
from execise_7 import execise_7
from execise_8 import execise_8
import geopandas as gpd
from matplotlib import animation
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from colorama import Fore, Back, Style


def exercise5():
    angle = 0
    step = 1
    fig = plt.figure()
    obstacle = np.array([[0, 0], [0, 2], [1, 2]])
    robot = -obstacle
    exo5 = execise_5(robot, obstacle)
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
    print("done!")
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
    exo_choice = input("choose en exercise from 5,7,8: ")
    try:
        exo_choice = int(exo_choice)
    except:
        print("Please choose from 5,6 or 8")
        exit()
    if exo_choice == 5:
        exercise5()
    elif exo_choice == 7:
        n = 3
        link = []
        angles = []
        endpoints = []
        try:
            link = input(
                "Please enter length of links seperate by space for examp 3 4 5: \n")
            link = list(map(int, link.strip().split()))[:n]
            if (len(link) != n):
                print(Fore.RED +
                      "Not right input. This is a three links problem, Please enter 3 numbers")
                exit()
        except:
            print(Fore.RED + "Something went wrong please check your inputs")
            exit()
        try:
            angles = input(
                "Please enter angles  as follow  3 4 5, \nleave empty for no angle(part a) but must enter endpoints: \n")
            angles = list(map(int, angles.strip().split()))[:n]
        except:
            print(Fore.RED + "Something went wrong plase check your inputs")
            exit()
        try:
            endpoints = input(
                "Please endpoints coordinate including the sum of angle pgi as follow  3 4 180, \nleave empty for endpoint(part b) but must enter angles : \n")
            endpoints = list(map(int, endpoints.strip().split()))[:n]
            if (len(endpoints) != n and not angles):
                print(
                    Fore.RED + "Not right input. Please include x, y ans sum of angle phi.")
                exit()
        except:
            print(Fore.RED + "Something went wrong plase check your inputs")
            exit()
        execise7(link, angles, endpoints)

    elif exo_choice == 8:
        trix = ""
        triangles = []
        while (1):
            n = input(
                "How many sides does the obstacle has?:  if done enter q: \n")
            if n == 'q':
                break
            n = int(n)
            trix = input(
                "Please enter x coordinates of polygon obstacle as follow 1 2 3. \n")

            trix = list(map(float, trix.strip().split()))[:n]

            triy = input(
                "Please enter y coordinates of polygon obstacle as follow 1 2 3: \n")
            triy = list(map(float, triy.strip().split()))[:n]
            triangles.append([trix, triy])

        n = input(
            "How many links?: \n")
        n = int(n)

        link = input(
            "Please enter links as follow with space 3 4: \n")
        link = list(map(float, link.strip().split()))[:n]
        # exo8 = execise_8([3, 4], 2, [[[3, 5, 7], [3, 1, 2]]])
        exo8 = execise_8(link, n, triangles)
        exo8.compute()
