"""Animate cooperative pursuit evasion simulation"""

import argparse
import pandas
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

from scipy.interpolate import griddata

import numpy as np

import os

import sys
root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..")
sys.path.append(os.path.join(root_dir, "build/src/main/"))
import coopPE_pb2


def main():
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.set_xlim(-0.1, 1.1)
    ax.set_ylim(-0.1, 1.1)
    ax.set_aspect("equal")

    ax.clear()
    ax.patch.set_facecolor('black')
    ax.set_xlim(-0.1, 1.1);
    ax.set_ylim(-0.1, 1.1);
    ax.set_xticks(np.linspace(0, 1, 10))
    ax.set_yticks(np.linspace(0, 1, 10))
    plt.tick_params(
        axis='both',
        which='both',
        bottom='off',
        left='off',
        top='off',
        labelbottom='off',
        labelleft='off')

    ax.add_patch(patches.Rectangle(
        (0.0, 0.0), 1.0, 1.0, alpha = 1.0, facecolor = "#333333"))

    ax.grid()

    ax.plot(1, 1, marker = "D", linestyle = "", c = "red", ms = 13)

    ax.plot([0, 1./9., 0], [0, 0, 1./9.], marker = "s", linestyle = "",
            c = "white", ms = 13)

    ax.plot([0, 7./9., 0], [0, 0, 7./9.], marker = "x", linestyle = "",
            c = "green", ms = 10, mew = 3)

    plt.savefig("../data/figures/sim_setup.png")
    plt.close(fig)

if __name__ == "__main__":
    main()
