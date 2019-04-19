# visualize.py
# Visualization of various RRT variations

import matplotlib.lines as lines
import matplotlib.patches as patches
import matplotlib.pyplot as plt

import algorithm.rrt as rrt
from structs.cartesian import *
from structs.tree import *


def draw_objects(axes: plt.Axes, objects: list):
    for obj in objects:
        x_bounds = obj.bounds['x']
        y_bounds = obj.bounds['y']
        x = x_bounds[0]
        w = x_bounds[1] - x
        y = y_bounds[0]
        h = y_bounds[1] - y
        axes.add_patch(patches.Rectangle((x, y), w, h, color='gray'))


def visualize_rrt():
    bounds = Space(x=(0, 10), y=(0, 10))

    start_point = Point(x=1, y=1)
    goal_point = Point(x=9, y=9)

    objects = [
        Space(x=(4, 8), y=(2, 4)),
        Space(x=(2, 4), y=(6, 10)),
        Space(x=(6, 8), y=(4, 8)),
        Space(x=(0, 2), y=(2, 4))
    ]

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    fig.set_size_inches(6, 6)
    ax1.set_title('Basic RRT')
    ax2.set_title('RRT-multi')
    ax3.set_title('RRT-connect')
    for axis in [ax1, ax2, ax3]:
        axis.set_xlim(bounds.bounds['x'])
        axis.set_ylim(bounds.bounds['y'])
        draw_objects(axis, objects)
    ax4.axis('off')

    for alg, axis in zip([rrt.rrt, rrt.rrt_multi, rrt.rrt_connect], [ax1, ax2, ax3]):
        _, goal_node = alg(start_point, goal_point, bounds, 0.3, limit=3000, obstacles=objects, ax=axis)

        if not goal_node:
            print('Timeout')
            return

        trace, cost = traceback(goal_node)
        print('Path found containing {} nodes with cost of {}'.format(len(trace), cost))

        trace_lines = [[tuple(trace[i].data.coordinates.values()),
                        tuple(trace[i + 1].data.coordinates.values())]
                       for i in range(len(trace) - 1)]

        for line in trace_lines:
            (line_x, line_y) = zip(*line)
            axis.add_line(lines.Line2D(line_x, line_y, linewidth=2, color='blue'))

    plt.plot()
    plt.show()


if __name__ == '__main__':
    visualize_rrt()
