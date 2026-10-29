
"""TEMPLATE FOR ALL GROUP MEMBERS: LUNL radiation-aware pathfinding.

SETUP:
  1. Put this file next to map_utils.py and the data/ folder.
  2. Install Pillow once: python -m pip install Pillow
  3. Rename this file (for example, astar.py or genetic.py).
  4. Implement find_route() below. Do not change the shared scoring rules.
  5. Run your algorithm file in PyCharm.
"""

import time

from map_utils import (
    MAP_SIZE,
    create_grid,
    load_grid,
    get_neighbors,
    is_valid_move,
    evaluate_path,
    choose_positions,
)

# Set to True for final group comparisons after generating shared_grid.json
# with map_utils.py. False generates a grid using the current MAP_SIZE.
USE_SAVED_GRID = False
NUMBER_OF_GOALS = 3


def find_route(grid, start, goals):
    """YOUR ALGORITHM GOES HERE.

    Parameters:
        grid: 2D list. -1 = obstacle; positive number = radiation cost.
        start: (row, col) tuple.
        goals: list of (row, col) tuples; visit EVERY goal in any order.

    Return:
        path: list of (row, col) tuples, starting at start and visiting
              all goals; each step must be up/down/left/right.

    Functions:
        get_neighbors(grid, row, col) -> adjacent traversable cells
        is_valid_move(grid, row, col) -> True/False
        evaluate_path(grid, path) -> (total_exposure, travel_time)

    Cost rules:
        The start cell counts once; revisiting a cell adds its cost again.
        Each move takes one time unit. No return to start is required - Might implement later ?
    """


def main():
    # Everyone uses the same map settings and deterministic start/goals.
    if USE_SAVED_GRID:
        grid = load_grid()
    else:
        grid = create_grid(MAP_SIZE)

    start, goals = choose_positions(grid, NUMBER_OF_GOALS)

    print("MAP:")
    for row in grid:
        print(" ".join(" XX" if value == -1 else f"{value:3}" for value in row))
    print("Map size:", len(grid), "x", len(grid[0]))
    print("Start:", start)
    print("Goals:", goals)

    # Time ONLY the algorithm, not loading or generating the map.
    start_time = time.perf_counter()
    path = find_route(grid, start, goals)
    execution_time = time.perf_counter() - start_time

    # Check the route before reporting results.
    if not path:
        print("No route returned")
        return
    if path[0] != start:
        raise ValueError("Path must begin at the starting position")
    if not set(goals).issubset(set(path)):
        raise ValueError("Path does not visit every goal")

    exposure, travel_time = evaluate_path(grid, path)
    print("\nFINAL RESULTS")
    print("Path:", path)
    print("Radiation exposure:", exposure)
    print("Travel time (moves):", travel_time)
    print("Execution time (seconds):", execution_time)
    # Optional: print your algorithm's nodes explored / generations here.


if __name__ == "__main__":
    main()

