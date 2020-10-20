from src.exercise2 import exercise2
import matplotlib.pyplot as plt


if __name__ == "__main__":
    center = [[4, 1], [7, -1]]
    start = [0, 0]
    goal = [10, 0]
    length = 1
    exo2 = exercise2(start, goal, center, length)
    obs= exo2.make_obs()
    for i in range(0, len(obs)):
        plt.plot(obs[i][0], obs[i][1])
    plt.xlim([-20, 20])
    plt.ylim([-20, 20])
    plt.show()
    # print(len(ret[1][1]))
