from src.exercise2 import exercise2
import matplotlib.pyplot as plt


if __name__ == "__main__":
    center = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    length = 1
    exo2 = exercise2(start, goal, center, length)
    exo2.compute()
    # for i in range(0, len(ret)):
    #     plt.plot(ret[i][0], ret[i][1])
    # plt.xlim([-20, 20])
    # plt.ylim([-20, 20])
    # plt.show()
    # print(len(ret[1][1]))
