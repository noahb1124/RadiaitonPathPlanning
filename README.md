# Radiation-Aware Robot Pathfinding

CSC2400 Algorithms Term Project

## Project overview

This project compares algorithms for planning an autonomous inspection robot's route through an environment with obstacles and radiation measurements. The robot starts at one cell, must visit every inspection goal, and does not need to return to the start.

Our primary objective is to **minimize cumulative radiation cost**. When two routes have equal radiation cost, we prefer the route with **fewer moves**. We also compare algorithm execution time and search effort.

Algorithms:
- **Brute force:** DFS
- **A\*:**
- **Genetic algorithm:**

## Intended Finished Project Structure

```text
RadiationPathfinding/
├── data/
│   ├── LUNL_radiation_data.txt
│   ├── LUNL_SLAM_map.pgm
│   └── LUNL_SLAM_map.yaml
├── map_utils.py
├── brute_force.py
├── astar.py
├── genetic.py
└── shared_grid.json         # Generated when map_utils.py saves a map
``

## Setup

1. Open the project folder in **PyCharm** (or another Python environment).
2. Make sure the three LUNL files are in the `data/` folder shown above.
3. Open the terminal and install Pillow:

   python -m pip install Pillow

4. Run `map_utils.py` to generate and display a grid and save `shared_grid.json`.
5. Run `brute_force.py`, or copy `algorithm_template.py` into `astar.py` / `genetic.py` and implement the algorithm.

> `map_utils.py` imports Pillow (`from PIL import Image`), so Pillow is required even if your algorithm only loads a saved JSON grid through that module.

## Map size and generation

Change the variable near the top of `map_utils.py`:

```python
MAP_SIZE = 5
```

`MAP_SIZE = X` produces an **X × X** grid. The `create_grid()` function processes the LUNL SLAM image and radiation readings into this simplified grid. Given identical dataset files and settings, generation is deterministic.

**Important:** Changing `MAP_SIZE` does **not** automatically update the saved grid. Run `map_utils.py` again to regenerate and save `shared_grid.json`. Saving to the same filename overwrites the previous version. Share the updated JSON file with teammates working on separate computers.

### Grid values

- `-1`: blocked/unknown cell (cannot be traversed)
- Positive integer: estimated radiation traversal cost; higher means more costly
- Coordinates: `(row, column)`, starting at `(0, 0)` in the upper-left corner
- Movement: up, down, left, or right; **no diagonals**

Radiation costs are estimated from recorded detector counts and are **not calibrated radiation doses**. A coarse cell classified as traversable is an approximation, not a guarantee of physical robot clearance.

## Shared functions in `map_utils.py`

| Function | Purpose |
| --- | --- |
| `create_grid(size=MAP_SIZE)` | Generate a square grid from the LUNL data |
| `save_grid(grid)` | Write the grid to `shared_grid.json` |
| `load_grid()` | Read a previously saved grid |
| `is_valid_move(grid, row, col)` | Check map bounds and obstacles |
| `get_neighbors(grid, row, col)` | Get traversable orthogonal neighbors |
| `choose_positions(grid, number_of_goals=3)` | Select a reachable start and spread-out goals |
| `evaluate_path(grid, path)` | Validate a path and calculate exposure and move count |

### How starting positions and goals are chosen

`choose_positions()` finds connected traversable regions using BFS and uses the largest region. It chooses the topmost, then leftmost cell in that region as the start. Goals are selected deterministically to be spread apart based on shortest-path movement distances. Goal selection does **not** use radiation costs.

The same grid and goal count produce the same start and goals.

## For team members: implementing an algorithm

Copy `algorithm.py` and rename it to your algorithm's filename. Implement the template's `find_route(grid, start, goals)` function, which should return an ordered list of coordinates including the start and all goals.

Example route format:

```python
[(0, 0), (0, 1), (1, 1), (2, 1)]
```

The returned route must move only between adjacent traversable cells. Revisiting cells is allowed and incurs radiation cost again. The robot may stop after reaching the final goal.

Useful imports:

```python
from map_utils import (
    MAP_SIZE, create_grid, load_grid, choose_positions,
    get_neighbors, is_valid_move, evaluate_path
)
```

**Important:** Use the same scoring rules in all implementations:

1. Minimize total radiation cost, including the starting cell and every revisit.
2. Break ties by minimizing the number of moves (`len(path) - 1`).
3. Record computation time separately from simulated travel time.

## Fair experiment procedure

For **each map size**:

1. Set `MAP_SIZE` in `map_utils.py`.
2. Run `map_utils.py` to generate and save that size's `shared_grid.json`.
3. Distribute the exact same `shared_grid.json` to all teammates.
4. Have each algorithm load the saved grid and call `choose_positions(grid)` with the same goal count.
5. Record route validity, radiation cost, move count, execution time, and search effort where available.
6. Repeat for other map sizes. Keep a copy of each experiment's grid if you want to reproduce earlier results.

or the final experiment, prefer loading the saved grid instead of independently regenerating it inside each algorithm.

## Notes and limitations

- Brute-force search can become extremely slow on larger maps. This is expected.
- The grid is a simplified model of the LUNL environment. Radiation is estimated for cells without direct readings.