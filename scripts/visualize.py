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

def evader_frame_data(time, rep, sims):
    evader = sims.reps[rep].time_points[time].evader

    return sims.network.nodes[evader.loc].x, sims.network.nodes[evader.loc].y


def pursuer_frame_data(time, rep, sims):
    pursuers = sims.reps[rep].time_points[time].pursuers

    pursuers_x = [sims.network.nodes[unit.loc].x for unit in pursuers]
    pursuers_y = [sims.network.nodes[unit.loc].y for unit in pursuers]

    return pursuers_x, pursuers_y


def post_frame_data(time, rep, sims):

    x = [node.x for node in sims.network.nodes]
    y = [node.y for node in sims.network.nodes]

    z = sims.reps[rep].time_points[time].posterior

    npoints = 50
    xi = np.linspace(0, 1, npoints)
    yi = np.linspace(0, 1, npoints)

    xi, yi = np.meshgrid(xi, yi)

    zi = griddata((x, y), z, (xi, yi), method = "cubic")

    zi = np.asarray(zi)
    zi = zi.reshape((npoints, npoints))

    return xi, yi, zi


def informant_data(time, rep, sims):
    informant = sims.reps[rep].time_points[time].informant
    if informant.has_tip:
        min_x = min(sims.network.nodes[i].x for i in informant.locs)
        min_y = min(sims.network.nodes[i].y for i in informant.locs)
        max_x = max(sims.network.nodes[i].x for i in informant.locs)
        max_y = max(sims.network.nodes[i].y for i in informant.locs)

        buf = 0.02

        x = min_x - buf
        y = min_y - buf
        w = max_x - min_x + 2 * buf
        h = max_y - min_y + 2 * buf


        return True, ((x, y), w, h)
    else:
        return True, None



def update_frame(index, sims, points, fig, ax):
    if index < len(points):
        # rep = points["rep"].iloc[index]
        # time = points["time"].iloc[index]
        rep = points[index][0]
        time = points[index][1]

        ax.clear()
        ax.patch.set_facecolor('black')
        ax.set_xlim(-0.1, 1.1);
        ax.set_ylim(-0.1, 1.1);
        ax.set_xticks(np.linspace(0, 1, 10))
        ax.set_yticks(np.linspace(0, 1, 10))
        ax.set_title("Rep: %04d Time: %04d" % (rep, time))
        plt.tick_params(
            axis='both',
            which='both',
            bottom='off',
            left='off',
            top='off',
            labelbottom='off',
            labelleft='off')

        x, y, z = post_frame_data(time, rep, sims)
        ax.contourf(x, y, z, 10, cmap = plt.cm.viridis,
                    vmax = np.max(z), vmin = np.min(z))

        ax.grid()

        has_tip, tip_data = informant_data(time, rep, sims)

        if tip_data is not None:
            ax.add_patch(patches.Rectangle(
                tip_data[0], tip_data[1], tip_data[2],
                alpha = 0.5, edgecolor = "black", facecolor = "red"))

        x, y = evader_frame_data(time, rep, sims)
        ax.plot(x, y, marker = "D", linestyle = "", c = "red")

        x, y = pursuer_frame_data(time, rep, sims)
        ax.plot(x, y, marker = "s", linestyle = "", c = "white")

    else:
        plt.close(fig)

    return ax


def update_frame_sidebyside(index, sims, points, fig, ax_left, ax_right):
    if index < len(points):
        rep = points[index][0]
        time = points[index][1]

        ax_left.clear()
        ax_left.patch.set_facecolor('black')
        ax_left.set_xlim(-0.1, 1.1);
        ax_left.set_ylim(-0.1, 1.1);
        ax_left.set_xticks(np.linspace(0, 1, 10))
        ax_left.set_yticks(np.linspace(0, 1, 10))

        ax_left.tick_params(
            axis='both',
            which='both',
            bottom='off',
            left='off',
            top='off',
            labelbottom='off',
            labelleft='off')

        ax_right.clear()
        ax_right.patch.set_facecolor('black')
        ax_right.set_xlim(-0.1, 1.1);
        ax_right.set_ylim(-0.1, 1.1);
        ax_right.set_xticks(np.linspace(0, 1, 10))
        ax_right.set_yticks(np.linspace(0, 1, 10))

        ax_right.tick_params(
            axis='both',
            which='both',
            bottom='off',
            left='off',
            top='off',
            labelbottom='off',
            labelleft='off')

        fig.suptitle("Rep: %04d Time: %04d" % (rep, time))

        x, y, z = post_frame_data(time, rep, sims)
        ax_right.contourf(x, y, z, 10, cmap = plt.cm.viridis,
                    vmax = np.max(z), vmin = np.min(z))

        ax_right.grid()

        ax_left.add_patch(patches.Rectangle(
            (0.0, 0.0), 1.0, 1.0, alpha = 1.0, facecolor = "#333333"))
        ax_left.grid()

        has_tip, tip_data = informant_data(time, rep, sims)

        if tip_data is not None:
            ax_left.add_patch(patches.Rectangle(
                tip_data[0], tip_data[1], tip_data[2],
                alpha = 0.5, edgecolor = "black", facecolor = "red"))

        x, y = evader_frame_data(time, rep, sims)
        ax_left.plot(x, y, marker = "D", linestyle = "", c = "red")

        x, y = pursuer_frame_data(time, rep, sims)
        ax_left.plot(x, y, marker = "s", linestyle = "", c = "white")

    else:
        plt.close(fig)

    return fig


def create_animation(points, sims):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.set_xlim(-0.1, 1.1)
    ax.set_ylim(-0.1, 1.1)
    ax.set_aspect("equal")

    ani = animation.FuncAnimation(fig, update_frame,
                                  frames = len(points),
                                  fargs = (sims, points, fig, ax),
                                  interval = 1000, blit = False)

    return ani


def create_animation_sidebyside(points, sims):
    fig = plt.figure()
    ax_left = fig.add_subplot(121)

    ax_left.set_xlim(-0.1, 1.1)
    ax_left.set_ylim(-0.1, 1.1)
    ax_left.set_aspect("equal")

    ax_right = fig.add_subplot(122)

    ax_right.set_xlim(-0.1, 1.1)
    ax_right.set_ylim(-0.1, 1.1)
    ax_right.set_aspect("equal")

    ani = animation.FuncAnimation(fig, update_frame_sidebyside,
                                  frames = len(points),
                                  fargs = (sims, points, fig,
                                           ax_left, ax_right),
                                  interval = 1000, blit = False)

    return ani


def main(sims_file, sidebyside, save_file, save_separate):
    sims = coopPE_pb2.SimStudyData()
    with open(sims_file, "r") as f:
        sims.ParseFromString(f.read())

    if save_file:
        if save_separate:
            ## save each replication individually
            file_base, file_ext = os.path.splitext(save_file)
            for i in range(len(sims.reps)):

                ## isolate points for just rep i
                points = []
                for j in range(len(sims.reps[i].time_points)):
                    points.append((i,j))

                if sidebyside:
                    ani = create_animation_sidebyside(points, sims)
                else:
                    ani = create_animation(points, sims)

                ## save with a counter
                ani.save(file_base + ("_%03d" % i) + file_ext,
                         writer='imagemagick', fps=1)
        else:
            ## gather points for all repications
            points = []
            for i in range(len(sims.reps)):
                for j in range(len(sims.reps[i].time_points)):
                    points.append((i,j))

            if sidebyside:
                ani = create_animation_sidebyside(points, sims)
            else:
                ani = create_animation(points, sims)

            ## write to file
            ani.save(save_file, writer='imagemagick', fps=1)
    else:
        ## gather points for all repications
        points = []
        for i in range(len(sims.reps)):
            for j in range(len(sims.reps[i].time_points)):
                points.append((i,j))

        if sidebyside:
            ani = create_animation_sidebyside(points, sims)
        else:
            ani = create_animation(points, sims)

        ## display to screen
        plt.show()

 
if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", type = str, required = True,
                    help = "directory with simulation results")
    ap.add_argument('--save', dest='save', action='store_true')
    ap.add_argument('--no-save', dest='save', action='store_false')
    ap.add_argument('--separate', dest='separate', action='store_true')
    ap.add_argument('--no-separate', dest='separate', action='store_false')
    ap.add_argument('--sidebyside', dest='sidebyside', action='store_true')
    ap.add_argument('--no-sidebyside', dest='sidebyside', action='store_false')
    ap.set_defaults(save=False, separate=False, sidebyside=False)

    args = ap.parse_args()

    if args.save:
        save_file = os.path.join(args.data_dir, "animation.gif")
    else:
        save_file = None

    main(os.path.join(args.data_dir, "sims.pb"),
         args.sidebyside, save_file, args.separate)
