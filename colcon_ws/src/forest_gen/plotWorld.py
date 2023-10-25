import matplotlib.pyplot as plt
import csv
import argparse
import numpy as np



class Cylinder:
    def __init__(self, x, y, r, h):
        self.x = x
        self.y = y
        self.r = r
        self.h = h

    def get_path(self, N=100):

        th = np.linspace(0, 2*np.pi, N)

        xs = self.x + self.r * np.cos(th)
        ys = self.y + self.r * np.sin(th)

        return xs, ys

def plotWorld(cylinders):

    plt.figure()
    for c in cylinders:
        xs, ys = c.get_path()
        plt.plot(xs, ys, color="black")

if __name__=="__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("filename")

    args = parser.parse_args()

    print(f"PROCESSING FILE : {args.filename}")

    f = open(args.filename, "r")

    cylinders = []

    reader = csv.reader(f)
    for (i, row) in enumerate(reader):
        if not i == 0:
            c = Cylinder(
                    float(row[0]),
                    float(row[1]),
                    float(row[2]),
                    float(row[3])
                    )
            print(c.x, c.y, c.r, c.h)
            cylinders.append(c)

    plotWorld(cylinders)

    plt.show()


