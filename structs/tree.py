# tree.py
# Tree data structure


class Tree:
    """ Simple tree data structure """

    root = None
    all_nodes = None

    def __init__(self, root=None):
        if root:
            self.root = root
        else:
            self.root = TreeNode()
        self.all_nodes = [self.root]


class TreeNode:
    """ Tree node data structure """

    parent = None
    children = []
    data = None
    cost = 0

    def __init__(self, parent=None, data=None, cost=0):
        self.parent = parent
        self.data = data
        self.cost = cost


def traceback(node: TreeNode):
    trace = [node]
    cost = 0
    while trace[0].parent:
        cost += trace[0].data.dist(trace[0].parent.data)
        trace.insert(0, trace[0].parent)
    return trace, cost
