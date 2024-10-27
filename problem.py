
class Grid:
    def __init__(self, grid_dimensions, initial_pos, target_positions, walls):
        self.initial_state = initial_pos
        self.target_states = self._read_target_pos(target_positions)
        self.walls = self._read_walls(walls)
        self.dimensions = grid_dimensions

    @staticmethod
    def _neighbor_nodes(node_state):
        x_cor = node_state[0]
        y_cor = node_state[1]
        neighbors = [["up", (x_cor, y_cor - 1)], ["left", (x_cor - 1, y_cor)], ["down", (x_cor, y_cor + 1)], ["right", (x_cor + 1, y_cor)]]
        return neighbors

    @staticmethod
    def _read_target_pos(targets):
        goals_list = targets.split("|")
        result = []
        for goal_state in goals_list:
            result.append(eval(goal_state))
        return result

    @staticmethod
    def _read_walls(walls):
        wall_coors = [eval(line.strip()) for line in walls]
        result = set()
        for wall in wall_coors:
            for i in range(wall[2]):
                for j in range(wall[3]):
                    result.add((wall[0] + i, wall[1] + j))
        return result

    def valid_squares(self, node_state):
        neighbors = self._neighbor_nodes(node_state)
        return [i for i in neighbors if ((0 <= i[1][0] < self.dimensions[1]) and
                                         (0 <= i[1][1] < self.dimensions[0]) and
                                         i[1] not in self.walls)]
