import numpy as np
import matplotlib.pyplot as plt


class Environment:
    def __init__(self, x2, y2, z2):
        self.fig = plt.figure()
            

        self.x1 = 0
        self.x2 = x2
        self.y1 = 0
        self.y2 = y2
        self.z1 = 0
        self.z2 = z2

        self.x, self.y, self.z = np.indices((x2, y2, z2))

        self.ax.set_xlim([0, x2])
        self.ax.set_ylim([0, y2])
        self.ax.set_zlim([0, z2])

        self.redzones = []

    def addRedZone(self, x1, x2, y1, y2, z1, z2):
        zone = (
            (self.x >= x1)
            & (self.x < x2)
            & (self.y >= y1)
            & (self.y < y2)
            & (self.z >= z1)
            & (self.z < z2)
        )
        self.redzones.append(zone)

    def display(self):
        color = (1,0,0, 0.5)
        for zone in self.redzones:
            self.ax.voxels(zone, facecolors=color)
        plt.show()


if __name__ == "__main__":
    env = Environment(30, 10, 12)
    env.addRedZone(0,env.x2, 0,env.y2, 0, 2)
    env.display()
