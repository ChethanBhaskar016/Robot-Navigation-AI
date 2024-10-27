class Node:
    def __init__(self, state, parent_node=None, children_nodes=None, path_cost=0, depth=0, action=None):
        if children_nodes is None:
            children_nodes = []
        self.state = state
        self.parent_node = parent_node
        self.children_nodes = children_nodes
        self.path_cost = path_cost
        self.depth = depth
        self.action = action
        self.visited = False

    @staticmethod
    def expand(parent_node, neighbors, step_cost, visited, frontier):
        for action, state in neighbors:
            if state not in visited:
                child = Node(state)
                child.parent_node = parent_node
                child.path_cost = parent_node.path_cost + step_cost
                child.depth = parent_node.depth + 1
                child.action = action
                parent_node.children_nodes.append(child)
                frontier.append(child)
        return frontier




