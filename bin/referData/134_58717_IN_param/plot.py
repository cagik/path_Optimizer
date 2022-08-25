import math
from turtle import color
import matplotlib.pyplot as plt
import numpy as np


def PathReader(dir1, color1):
    f1 = open(dir1, "r")
    line1 = f1.readline()
    x, y = [], []
    while line1:
      linedate1 = line1.split(",")
      if line1 == "\n":
        break
      x.append(float(linedate1[0]))
      y.append(float(linedate1[1]))
      line1 = f1.readline()
    x.pop(0)
    y.pop(0)
    plt.plot(x, y, color1)

def test():
    x = np.arange(-100, 100, 0.1)
    y = []
    for t in x:
        y_tmp = 1 / (1 + math.exp(0.25 * t - 3))
        y.append(y_tmp)
    plt.plot(x, y, "r")

def main():
    # PathReader("./edge.txt", "go")
    # PathReader("./path.txt", "ro")
    PathReader("./edge_inertia.txt", "go")
    PathReader("./path_inertia_out.txt", "ro")
    plt.show()
main()
