from node import Node

STEP_COST = 1


class SearchAlgorithm:
    def __init__(self, problem):
        self.problem = problem

    def search(self, arg):
        visited, frontier = [], []
        if arg == "DFS":
            return self._dfs(frontier, visited)
        elif arg == "BFS":
            return self._bfs(frontier, visited)
        elif arg == "GBFS":
            return self._gbfs(frontier, visited)
        elif arg == "AS":
            return self._a_star(frontier, visited)
        elif arg == "IDDFS":
            return self._iddfs()
        elif arg == "IDAS":
            return self._idas()
        elif arg == "BFS+":
            return self._bfs_plus()
        else:
            return 0

    def _dfs(self, frontier, visited):
        new_node = Node(state=self.problem.initial_state)
        frontier.append(new_node)
        while len(frontier) > 0:
            current_node = frontier.pop()
            if current_node.state not in visited:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state in self.problem.target_states:
                return current_node.state, len(visited), self.get_path(current_node)
            current_node.expand(current_node, self.problem.valid_squares(current_node.state)[::-1]
                                , 1, visited, frontier)
        return False, len(visited)

    def _bfs(self, frontier, visited):
        new_node = Node(state=self.problem.initial_state)
        frontier.append(new_node)
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            if current_node.state not in visited:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state in self.problem.target_states:
                return current_node.state, len(visited), self.get_path(current_node)
            current_node.expand(current_node, self.problem.valid_squares(current_node.state)
                                , 1, visited, frontier)
        return False, len(visited)

    def _gbfs(self, frontier, visited):
        new_node = Node(state=self.problem.initial_state)
        frontier.append(new_node)
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            if current_node.state not in visited:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state in self.problem.target_states:
                return current_node.state, len(visited), self.get_path(current_node)
            current_node.expand(current_node, self.problem.valid_squares(current_node.state)
                                , 1, visited, frontier)
            frontier.sort(key=self.gbfs_evaluation_function)
        return False, len(visited)

    def _a_star(self, frontier, visited):
        new_node = Node(state=self.problem.initial_state)
        frontier.append(new_node)
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            if current_node.state not in visited:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state in self.problem.target_states:
                return current_node.state, len(visited), self.get_path(current_node)
            current_node.expand(current_node, self.problem.valid_squares(current_node.state)
                                , 1, visited, frontier)
            frontier.sort(key=self.astar_evaluation_function)
        return False, len(visited)

    def _idas(self):
        threshold = self.heuristic_function(self.problem.initial_state, self.problem.target_states)
        while True:
            result, num_visited, pruned = self.cost_limited_dfs_search(threshold)
            if result is not None:
                return (False, num_visited, None) if not result else (result.state, num_visited, self.get_path(result))
            if len(pruned) > 0:
                threshold = min([self.astar_evaluation_function(node) for node in pruned])

    def cost_limited_dfs_search(self, threshold):
        frontier = [Node(state=self.problem.initial_state)]
        visited = []
        pruned = set()
        result = False, len(visited), pruned
        while len(frontier) > 0:
            current_node = frontier.pop()
            if current_node.state not in visited and current_node not in pruned:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state in self.problem.target_states:
                return current_node, len(visited), pruned
            current_heuristic = self.astar_evaluation_function(current_node)
            if current_heuristic <= threshold:
                current_node.expand(current_node, self.problem.valid_squares(current_node.state)[::-1]
                                    , 1, visited, frontier)
            else:
                pruned.add(current_node)
                result = None, len(visited), pruned
            frontier.sort(key=self.astar_evaluation_function, reverse=True)
        return result if result[0] is None else (False, len(visited), pruned)

    def _iddfs(self):
        depth = 0
        while True:
            result, num_visited = self.depth_limited_dfs_search(depth)
            if result is not None:
                return (False, num_visited, None) if not result else (result.state, num_visited, self.get_path(result))
            depth += 1

    def depth_limited_dfs_search(self, depth_limit):
        frontier = [Node(state=self.problem.initial_state)]
        visited = []
        result = False, len(visited)
        while len(frontier) > 0:
            current_node = frontier.pop()
            if current_node.state not in visited:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state in self.problem.target_states:
                return current_node, len(visited)
            if current_node.depth < depth_limit:
                current_node.expand(current_node, self.problem.valid_squares(current_node.state)[::-1]
                                    , 1, visited, frontier)
            else:
                result = None, len(visited)
        return result if result[0] is None else (False, len(visited))

    def _bfs_plus(self):
        goals_dictionary = {}
        goal_states = self.problem.target_states
        goal_state = self.problem.initial_state
        for goal in goal_states:
            goals_dictionary[goal] = self.distance_to_goal(node_state=goal_state, goal=goal)
        goal_states = []
        goals_dictionary = dict(sorted(goals_dictionary.items()))
        for goal in goals_dictionary:
            goal_states.append(goal)
        solution = []
        while len(goal_states) > 0:
            frontier = [Node(goal_state)]
            visited = []
            current_state = goal_state
            goal_state = goal_states.pop(0)
            solution.append(self._modified_bfs(current_state, goal_state, visited, frontier))
        return solution

    def _modified_bfs(self, initial_state, goal_state, visited, frontier):
        frontier.insert(0, Node(initial_state))
        while len(frontier) > 0:
            current_node = frontier.pop(0)
            if current_node.state not in visited:
                visited.append(current_node.state)
            else:
                continue
            if current_node.state == goal_state:
                return current_node.state, self.get_path(current_node)
            current_node.expand(current_node, self.problem.valid_squares(current_node.state)
                                , 1, visited, frontier)
        return False, None

    @staticmethod
    def get_path(current_node):
        # path to the nearest goal node
        path = []
        parent = current_node.parent_node
        while parent is not None:
            path.append(current_node.action)
            current_node = parent
            parent = current_node.parent_node
        return path[::-1]

    def heuristic_function(self, node_state, goal_states):
        min_distance = float('inf')
        for goal_state in goal_states:
            total_distance = self.distance_to_goal(node_state, goal_state)
            min_distance = min(total_distance, min_distance)  # Update min_distance with the minimum value
        return min_distance

    @staticmethod
    def distance_to_goal(node_state, goal):
        return abs((node_state[0] - goal[0]) + (node_state[0] - goal[0]))

    def gbfs_evaluation_function(self, node):
        return self.heuristic_function(node.state, self.problem.target_states)

    def astar_evaluation_function(self, node):
        return self.heuristic_function(node.state, self.problem.target_states) + node.path_cost
