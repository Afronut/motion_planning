from src.util import utils
from src.phase1 import path as path1
from src.phase2 import path as path2
from src.phase3 import path as path3

import matplotlib.pyplot as plt


def phase_1(tree=False):
    path1(tree)


def phase_2(tree=False):
    path2(tree)


def phase_3(tree=False):
    path3(tree)


if __name__ == "__main__":
    inp = input("Enter phase to run (1,2 or 3): ")
    try:
        inp = int(inp)
    except:
        print("Please enter an interger")
        exit(0)
    tree = input("Show tree? (yes/no): ")
    if tree == "yes":
        tree = True
    else:
        tree = False
    if (inp == 1):
        phase_1(tree)
    elif (inp == 2):
        phase_2(tree)
    elif (inp == 3):
        phase_3(tree)

    # const = ph1.make_const()
    # print(const)
    # plt.show()
