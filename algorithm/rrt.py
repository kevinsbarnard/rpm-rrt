# rrt.py
# RRT algorithm implementation

import matplotlib.lines as lines

from algorithm.nearest_neighbor import *
from structs.cartesian import *
from structs.tree import *


def new_conf(q_near: Point, q_samp: Point, delta_q: float):
    q_vec = q_samp - q_near
    q_vec_scaled = q_vec / q_vec.norm() * delta_q
    return q_near + q_vec_scaled


def valid(pt: Point, bounds: Space, objects: list):
    """ Ensure point to be placed is valid and not in collision """
    return bounds.within(pt) and not any([obj.within(pt) for obj in objects])


def rrt(start: Point, goal: Point, bounds: Space, epsilon: float, limit=5000, obstacles=None, ax=None, color='red'):
    """
    Basic RRT algorithm.
    :param start: Starting point
    :param goal: Goal point
    :param bounds: Bounds of the configuration space
    :param epsilon: Maximum distance to move between points
    :param limit: Number of iterations/nodes to be added
    :param obstacles: Obstacles in configuration space
    :param ax: Axes object for visualization
    :param color: Tree color
    :return: RRT and goal node if found
    """

    # Ensure start and end have same dimension
    if start.dim != goal.dim:
        raise ValueError('Start point dimension ({}) and goal point dimension ({}) are must be equal'.format(start.dim, goal.dim))

    # Initialize RRT
    start_node = TreeNode(data=start)
    tree = Tree(start_node)

    # Iterate until limit reached
    for _ in range(limit):
        # Sample point
        q_samp = bounds.sample()

        # Find nearest neighbor in tree
        q_near = nearest_neighbor(q_samp,
                                  tree.all_nodes,
                                  key=lambda node: node.data)
        # Compute new point
        q_new = new_conf(q_near.data, q_samp, epsilon)

        # Check collision and add to tree
        if valid(q_new, bounds, obstacles):
            # Create new node in tree
            new_node = TreeNode(parent=q_near,
                                data=q_new,
                                cost=(q_near.cost + epsilon))
            q_near.children.append(new_node)
            tree.all_nodes.append(new_node)

            # Update visualization
            if ax:
                ax.add_line(lines.Line2D(
                    (q_near.data.coordinates['x'], q_new.coordinates['x']),
                    (q_near.data.coordinates['y'], q_new.coordinates['y']),
                    linewidth=1,
                    color=color))

            # Check if goal can be added, return success
            if goal.dist(q_new) < epsilon:
                goal_node = TreeNode(parent=new_node,
                                     data=goal,
                                     cost=(new_node.cost + goal.dist(q_new)))
                new_node.children.append(goal_node)
                tree.all_nodes.append(goal_node)
                return tree, goal_node

    # Return None if fail
    return tree, None


def rrt_multi(start: Point, goal: Point, bounds: Space, epsilon: float, limit=5000, obstacles=None, ax=None, color='red', n=3):
    """
    Semi-greedy informed RRT algorithm.
    Samples multiple points for growth.
    :param start: Starting point
    :param goal: Goal point
    :param bounds: Bounds of the configuration space
    :param epsilon: Maximum distance to move between points
    :param limit: Number of iterations/nodes to be added
    :param obstacles: Obstacles in configuration space
    :param ax: Axes object for visualization
    :param color: Tree color
    :param n: Number of points to sample
    :return: RRT and goal node if found
    """

    # Ensure start and end have same dimension
    if start.dim != goal.dim:
        raise ValueError('Start point dimension ({}) and goal point dimension ({}) are must be equal'.format(start.dim, goal.dim))

    # Initialize RRT
    start_node = TreeNode(data=start)
    tree = Tree(start_node)

    # Iterate until limit reached
    for _ in range(limit):
        # Sample n points and pick closest point to goal
        q_samp = bounds.sample(n=n)
        q_samp = min(q_samp,
                     key=lambda pt: pt.dist(goal))

        # Find nearest neighbor in tree
        q_near = nearest_neighbor(q_samp,
                                  tree.all_nodes,
                                  key=lambda node: node.data)

        # Compute new point
        q_new = new_conf(q_near.data, q_samp, epsilon)

        # Check collision and add to tree
        if valid(q_new, bounds, obstacles):
            # Create new node in tree
            new_node = TreeNode(parent=q_near,
                                data=q_new,
                                cost=(q_near.cost + epsilon))
            q_near.children.append(new_node)
            tree.all_nodes.append(new_node)

            # Update visualization
            if ax:
                ax.add_line(lines.Line2D(
                    (q_near.data.coordinates['x'], q_new.coordinates['x']),
                    (q_near.data.coordinates['y'], q_new.coordinates['y']),
                    linewidth=1,
                    color=color))

            # Check if goal can be added, return success
            if goal.dist(q_new) < epsilon:
                goal_node = TreeNode(parent=new_node,
                                     data=goal,
                                     cost=(new_node.cost + goal.dist(q_new)))
                new_node.children.append(goal_node)
                tree.all_nodes.append(goal_node)
                return tree, goal_node

    # Return None if fail
    return tree, None


def connect(f_node: TreeNode, b_node: TreeNode):
    """ Recursively connect nodes from a forward tree to the root of the backward tree """
    if not b_node:
        return
    connect(b_node, b_node.parent)
    b_node.parent = f_node


def rrt_connect(start: Point, goal: Point, bounds: Space, epsilon: float, limit=5000, obstacles=None, ax=None, f_color='red', b_color='green'):
    """
    RRT-connect algorithm.
    :param start: Starting point
    :param goal: Goal point
    :param bounds: Bounds of the configuration space
    :param epsilon: Maximum distance to move between points
    :param limit: Number of iterations/nodes to be added
    :param obstacles: Obstacles in configuration space
    :param ax: Axes object for visualization
    :param f_color: Forward tree color
    :param b_color: Backward tree color
    :return: RRTs (forward, backward) and goal node if found
    """

    # Ensure start and end have same dimension
    if start.dim != goal.dim:
        raise ValueError('Start point dimension ({}) and goal point dimension ({}) are must be equal'.format(start.dim, goal.dim))

    # Initialize forward and backward RRTs
    start_node = TreeNode(data=start)
    goal_node = TreeNode(data=goal)
    forward_tree = Tree(start_node)
    backward_tree = Tree(goal_node)

    # Iterate until limit reached
    for _ in range(limit):
        # Grow both forward and backward by one
        for tree in [forward_tree, backward_tree]:
            # Sample point
            q_samp = bounds.sample()

            # Find nearest neighbor in tree
            q_near = nearest_neighbor(q_samp,
                                      tree.all_nodes,
                                      key=lambda node: node.data)

            # Compute new point
            q_new = new_conf(q_near.data, q_samp, epsilon)

            # Check collision and add to tree
            if valid(q_new, bounds, obstacles):
                # Create new node in tree
                new_node = TreeNode(parent=q_near,
                                    data=q_new,
                                    cost=(q_near.cost + epsilon))
                q_near.children.append(new_node)
                tree.all_nodes.append(new_node)

                # Update visualization
                if ax:
                    ax.add_line(lines.Line2D(
                        (q_near.data.coordinates['x'], q_new.coordinates['x']),
                        (q_near.data.coordinates['y'], q_new.coordinates['y']),
                        linewidth=1,
                        color=f_color if tree is forward_tree else b_color))

        # Check if trees can be connected, return success
        for f_node in forward_tree.all_nodes:
            for b_node in backward_tree.all_nodes:
                if b_node.data.dist(f_node.data) < epsilon:
                    connect(f_node, b_node)
                    return (forward_tree, backward_tree), goal_node

    # Return None if fail
    return (forward_tree, backward_tree), None
