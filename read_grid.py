from problem import Grid


class GridReader:
    def __init__(self, file):
        self.file = file

    def make_grid(self):
        grid_dimensions = eval(self.file.readline())
        initial_state = eval(self.file.readline())
        goal_state_str = self.file.readline()
        walls = []
        while True:
            wall = self.file.readline().strip()
            if wall == "":
                break
            walls.append(wall)

        grid = Grid(grid_dimensions=grid_dimensions, initial_pos=initial_state, target_positions=goal_state_str,
                    walls=walls)
        return grid
