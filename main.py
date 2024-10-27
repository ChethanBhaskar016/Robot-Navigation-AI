from read_grid import GridReader
from search_algorithm import SearchAlgorithm


def main_function(file_path, search_method):
    with open(file_path, "r") as robot_nav_file:
        grid_reader = GridReader(robot_nav_file)
        grid = grid_reader.make_grid()

    search = SearchAlgorithm(grid)
    path = search.search(search_method)
    print(f"{file_path} {search_method}")
    if search_method == "BFS+":
        result, direction = path[0]
        if not result:
            print(f"No goal is reachable")
        else:
            for i in path:
                result, direction = i
                if result:
                    print(f"<Node{result}>")
                    print(direction)
    else:
        if path == 0:
            print("Invalid Search Method")
        elif not path[0]:
            print(f"No goal is reachable; {path[1]}")
        else:
            print(f"<Node{path[0]}> {path[1]}")
            print(path[2])
