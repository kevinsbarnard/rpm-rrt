# evaluate.py
# Evaluation of various RRT algorithms


import argparse
import datetime
import os
import random
import sys
import time
from json import dump

from algorithm.rrt import rrt, rrt_connect, rrt_multi
from structs.cartesian import *
from structs.tree import *


def generate_obstacles(start: Point, goal: Point, n=5, size=1):
    obstacles = []
    corner_points = [
        Point(x=start.coordinates['x'] - size / 2, y=start.coordinates['y'] - size / 2),  # Bottom left
        Point(x=start.coordinates['x'] + size / 2, y=start.coordinates['y'] - size / 2),  # Bottom right
        Point(x=start.coordinates['x'] - size / 2, y=start.coordinates['y'] + size / 2),  # Top left
        Point(x=start.coordinates['x'] + size / 2, y=start.coordinates['y'] + size / 2),  # Top right
        Point(x=goal.coordinates['x'] - size / 2, y=goal.coordinates['y'] - size / 2),  # Bottom left
        Point(x=goal.coordinates['x'] + size / 2, y=goal.coordinates['y'] - size / 2),  # Bottom right
        Point(x=goal.coordinates['x'] - size / 2, y=goal.coordinates['y'] + size / 2),  # Top left
        Point(x=goal.coordinates['x'] + size / 2, y=goal.coordinates['y'] + size / 2)  # Top right
    ]
    for _ in range(n):
        valid = False
        obs = None
        while not valid:
            start_x = random.uniform(-10, 10)
            start_y = random.uniform(-10, 10)
            end_x = start_x + size
            end_y = start_y + size

            obs = Space(x=(start_x, end_x), y=(start_y, end_y))

            if not any([obs.within(point) for point in corner_points]):
                valid = True
        obstacles.append(obs)
    return obstacles


def run_evaluations_batch(epsilon: float, limit: int, batch: int, n_obstacles: int):
    print('Running {2} iteration{4} with epsilon={0}, limit={1}, n_obstacles={3}'.format(epsilon, limit, batch, n_obstacles, 's' if batch != 1 else ''))

    bounds = Space(x=(-10, 10), y=(-10, 10))
    start_point = Point(x=-9, y=-9)
    goal_point = Point(x=9, y=9)

    order = ['basic', 'connect', 'multi']
    data = {
        'nodes': {
            'basic': [],
            'connect': [],
            'multi': []
        },
        'times': {
            'basic': [],
            'connect': [],
            'multi': []
        },
        'costs': {
            'basic': [],
            'connect': [],
            'multi': []
        }
    }
    for iteration in range(batch):
        print('iteration={}/{}'.format(iteration + 1, batch), end='...')
        sys.stdout.flush()

        # Generate obstacles
        obstacles = generate_obstacles(start_point, goal_point, n=n_obstacles)

        # Run basic RRT
        t0 = time.time()
        basic_rrt, basic_goal = rrt(start_point, goal_point, bounds, epsilon, limit=limit, obstacles=obstacles)
        basic_t = time.time() - t0

        # Run RRT-connect
        t0 = time.time()
        connect_rrts, connect_goal = rrt_connect(start_point, goal_point, bounds, epsilon, limit=limit, obstacles=obstacles)
        connect_t = time.time() - t0

        # Run RRT-multi
        t0 = time.time()
        multi_rrt, multi_goal = rrt_multi(start_point, goal_point, bounds, epsilon, limit=limit, obstacles=obstacles)
        multi_t = time.time() - t0

        # Get number of nodes from trees
        basic_n_nodes = len(basic_rrt.all_nodes)
        connect_n_nodes = sum([len(tree.all_nodes) for tree in connect_rrts])
        multi_n_nodes = len(multi_rrt.all_nodes)

        # Compute path costs
        basic_cost, connect_cost, multi_cost = (None, None, None)
        if basic_goal:
            basic_trace, basic_cost = traceback(basic_goal)
        if connect_goal:
            connect_trace, connect_cost = traceback(connect_goal)
        if multi_goal:
            multi_trace, multi_cost = traceback(multi_goal)

        # Collect data
        node_data = [basic_n_nodes, connect_n_nodes, multi_n_nodes]
        time_data = [basic_t, connect_t, multi_t]
        cost_data = [basic_cost, connect_cost, multi_cost]
        for i, item in enumerate(order):
            data['nodes'][item].append(node_data[i])
            data['times'][item].append(time_data[i])
            data['costs'][item].append(cost_data[i])

        print('done')

    # Dump JSON data
    if not os.path.isdir('eval_data'):
        os.mkdir('eval_data')
    now = datetime.datetime.now()
    with open(os.path.join('eval_data', '_'.join([str(tm) for tm in [
        now.year,
        now.month,
        now.day,
        now.hour,
        now.minute,
        now.second
    ]]) + '.json'), 'w') as out_file:
        print('Dumping JSON...')
        dump(data, out_file)
        out_file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('epsilon', type=float, help='epsilon value for RRT algorithms')
    parser.add_argument('limit', type=int, help='iteration limit for RRT algorithms')
    parser.add_argument('batch', type=int, help='batch size (an integer)')
    parser.add_argument('n_obstacles', type=int, help='number of obstacles to generate (an integer)')
    args = parser.parse_args()
    run_evaluations_batch(args.epsilon, args.limit, args.batch, args.n_obstacles)
