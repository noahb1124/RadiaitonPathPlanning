
# IMPORTANT: PILLOW MUST BE INSTALLED FOR THIS PROGRAM TO RUN
# 1. Open the project in PyCharm
# 2. Open the terminal
# 3. Run "python -m pip install Pillow" in the terminal
# 4 Boom, you're done that was really easy

import csv
import json
import math
from pathlib import Path
from collections import deque
from PIL import Image


# IMPORTANT: MAP SIZE INITIALIZATION
# MAP_SIZE = Y CREATES A Y BY Y SQUARE GRID.
# CHANGE THIS VALUE TO GENERATE DIFFERENT MAP SIZES.
# LARGER GRIDS REQUIRE MORE PROCESSING TIME,
# ESPECIALLY FOR BRUTE-FORCE ALGORITHMS.

MAP_SIZE = 5

# LOCATION OF THE LUNL DATASET FILES.
# THE DATA FOLDER SHOULD BE IN THE SAME FOLDER AS map_utils.py.

DATA_DIR = Path(__file__).resolve().parent / "data"


# ------------------------------------------------------------
# LOAD RADIATION DATA
# ------------------------------------------------------------

# READ THE RADIATION MEASUREMENTS FROM THE LUNL DATASET.
# EACH MEASUREMENT CONTAINS AN X COORDINATE, Y COORDINATE,
# AND A RECORDED RADIATION COUNT.

def load_measurements(data_dir=DATA_DIR):

    measurements = []

    with open(Path(data_dir) / "LUNL_radiation_data.txt", newline="") as file:

        for row in csv.DictReader(file):
            measurements.append((
                float(row["x"]),
                float(row["y"]),
                float(row["counts"])
            ))

    if not measurements:
        raise ValueError("No radiation measurements were found")

    return measurements


# ------------------------------------------------------------
# LOAD MAP INFORMATION
# ------------------------------------------------------------

# READ THE MAP RESOLUTION AND ORIGIN FROM THE YAML FILE.
# RESOLUTION TELLS US HOW MUCH REAL-WORLD DISTANCE
# EACH PIXEL REPRESENTS.
# ORIGIN TELLS US WHERE THE MAP BEGINS IN WORLD COORDINATES.

def load_metadata(data_dir=DATA_DIR):

    text = (Path(data_dir) / "LUNL_SLAM_map.yaml").read_text()

    values = {}

    for line in text.splitlines():

        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()

    resolution = float(values["resolution"])

    origin = [
        float(v.strip())
        for v in values["origin"].strip("[]").split(",")
    ]

    return resolution, origin[0], origin[1]


# ------------------------------------------------------------
# CREATE THE RADIATION GRID
# ------------------------------------------------------------

# CONVERT THE LUNL SLAM MAP AND RADIATION DATA INTO
# A SMALLER SQUARE GRID THAT OUR ALGORITHMS CAN USE.
#
# -1 REPRESENTS AN OBSTACLE OR UNKNOWN AREA.
# POSITIVE NUMBERS REPRESENT ESTIMATED RADIATION COSTS.
#
# THE MAP IS DIVIDED INTO MAP_SIZE ROWS AND COLUMNS.
# A CELL IS CONSIDERED TRAVERSABLE IF ENOUGH OF ITS
# ORIGINAL MAP PIXELS REPRESENT FREE SPACE.
#
# RADIATION COSTS ARE ESTIMATED USING THE CLOSEST
# RECORDED RADIATION MEASUREMENT.
#
# THESE VALUES ARE EXPERIMENTAL COSTS, NOT ACTUAL
# CALIBRATED RADIATION DOSES.

def create_grid(size=MAP_SIZE, data_dir=DATA_DIR,
                margin=0.3, min_free_fraction=0.5):

    if size < 2:
        raise ValueError("size must be at least 2")

    readings = load_measurements(data_dir)

    resolution, origin_x, origin_y = load_metadata(data_dir)

    # OPEN THE ORIGINAL SLAM MAP AS A GRAYSCALE IMAGE.
    image = Image.open(
        Path(data_dir) / "LUNL_SLAM_map.pgm"
    ).convert("L")

    width, height = image.size

    # FIND THE AREA COVERED BY THE RADIATION MEASUREMENTS.
    # ADD A SMALL MARGIN AROUND THE MEASURED AREA.

    xs = [p[0] for p in readings]
    ys = [p[1] for p in readings]

    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin

    # CALCULATE THE REAL-WORLD WIDTH AND HEIGHT OF EACH CELL.

    dx = (max_x - min_x) / size
    dy = (max_y - min_y) / size

    grid = []

    # CREATE EACH ROW OF THE GRID.
    for row in range(size):

        result_row = []

        top_y = max_y - row * dy
        bottom_y = top_y - dy

        # CREATE EACH COLUMN WITHIN THE CURRENT ROW.
        for col in range(size):

            left_x = min_x + col * dx
            right_x = left_x + dx

            # CONVERT REAL-WORLD COORDINATES INTO
            # PIXEL COORDINATES ON THE ORIGINAL SLAM MAP.

            left_px = max(
                0, math.floor((left_x - origin_x) / resolution)
            )

            right_px = min(
                width, math.ceil((right_x - origin_x) / resolution)
            )

            top_px = max(
                0, math.floor(height - (top_y - origin_y) / resolution)
            )

            bottom_px = min(
                height, math.ceil(height - (bottom_y - origin_y) / resolution)
            )

            # MARK CELLS OUTSIDE THE IMAGE AS OBSTACLES.

            if left_px >= right_px or top_px >= bottom_px:
                result_row.append(-1)
                continue

            # EXAMINE THE ORIGINAL MAP PIXELS INSIDE THIS CELL.
            # PIXEL VALUE 254 REPRESENTS CONFIRMED FREE SPACE.

            pixels = image.crop((
                left_px, top_px, right_px, bottom_px
            ))

            pixel_values = pixels.tobytes()

            free_fraction = pixel_values.count(254) / len(pixel_values)

            # BLOCK CELLS THAT DO NOT CONTAIN ENOUGH FREE SPACE.

            if free_fraction < min_free_fraction:
                result_row.append(-1)
                continue

            # FIND THE CENTER OF THE CURRENT GRID CELL.

            center_x = (left_x + right_x) / 2
            center_y = (top_y + bottom_y) / 2

            # FIND THE CLOSEST RADIATION MEASUREMENT.

            nearest = min(
                readings,
                key=lambda p:
                (p[0] - center_x) ** 2 +
                (p[1] - center_y) ** 2
            )

            # ASSIGN AN ESTIMATED RADIATION COST.
            # THE ADDITIONAL 1 PENALIZES MOVEMENT EVEN
            # THROUGH CELLS WITH ZERO RECORDED COUNTS.

            result_row.append(1 + round(nearest[2]))

        grid.append(result_row)

    return grid


# ------------------------------------------------------------
# CHECK VALID MOVEMENTS
# ------------------------------------------------------------

# CHECK WHETHER A CELL IS INSIDE THE MAP AND NOT AN OBSTACLE.
# RETURNS TRUE IF THE ROBOT CAN ENTER THE CELL.

def is_valid_move(grid, row, col):

    return (
        0 <= row < len(grid)
        and 0 <= col < len(grid[0])
        and grid[row][col] != -1
    )


# ------------------------------------------------------------
# FIND NEIGHBORING CELLS
# ------------------------------------------------------------

# RETURN ALL VALID CELLS DIRECTLY ADJACENT TO A POSITION.
# ROBOTS CAN MOVE UP, DOWN, LEFT, OR RIGHT.
# DIAGONAL MOVEMENT IS NOT ALLOWED.

def get_neighbors(grid, row, col):

    neighbors = []

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]

    for dr, dc in directions:

        new_row = row + dr
        new_col = col + dc

        if is_valid_move(grid, new_row, new_col):
            neighbors.append((new_row, new_col))

    return neighbors


# ------------------------------------------------------------
# SELECT STARTING POSITION AND GOALS
# ------------------------------------------------------------

# AUTOMATICALLY SELECT A VALID STARTING POSITION AND GOALS.
#
# FIRST, FIND GROUPS OF CONNECTED TRAVERSABLE CELLS.
# SELECT THE LARGEST CONNECTED GROUP SO ALL GOALS
# CAN BE REACHED FROM THE START.
#
# THE START IS THE TOPMOST, THEN LEFTMOST CELL
# IN THE SELECTED GROUP.
#
# GOALS ARE CHOSEN TO BE SPREAD APART BASED ON
# SHORTEST-PATH MOVEMENT DISTANCE.
#
# THIS FUNCTION DOES NOT USE RANDOM NUMBERS.
# THE SAME GRID ALWAYS PRODUCES THE SAME POSITIONS.

def choose_positions(grid, number_of_goals=3):

    seen = set()
    components = []

    # FIND ALL CONNECTED GROUPS OF FREE CELLS.

    for row in range(len(grid)):
        for col in range(len(grid[0])):

            cell = (row, col)

            if not is_valid_move(grid, row, col) or cell in seen:
                continue

            component = []
            queue = deque([cell])
            seen.add(cell)

            # USE BREADTH-FIRST SEARCH TO FIND
            # EVERY CELL IN THIS CONNECTED GROUP.

            while queue:

                current = queue.popleft()
                component.append(current)

                for neighbor in get_neighbors(grid, *current):

                    if neighbor not in seen:
                        seen.add(neighbor)
                        queue.append(neighbor)

            components.append(component)

    if not components:
        raise ValueError("No traversable cells in grid")

    # CHOOSE THE LARGEST CONNECTED GROUP.

    largest = max(components, key=len)

    if len(largest) < number_of_goals + 1:
        raise ValueError(
            "Not enough connected cells for start and goals"
        )

    start = min(largest)
    selected = [start]

    # SELECT GOALS THAT ARE FAR APART FROM
    # THE POSITIONS ALREADY SELECTED.

    for _ in range(number_of_goals):

        distance_maps = []

        for origin in selected:

            distances = {origin: 0}
            queue = deque([origin])

            while queue:

                current = queue.popleft()

                for neighbor in get_neighbors(grid, *current):

                    if neighbor not in distances:
                        distances[neighbor] = distances[current] + 1
                        queue.append(neighbor)

            distance_maps.append(distances)

        candidates = [
            cell for cell in largest
            if cell not in selected
        ]

        goal = max(
            candidates,
            key=lambda cell: (
                min(distances[cell] for distances in distance_maps),
                -cell[0],
                -cell[1]
            )
        )

        selected.append(goal)

    return start, selected[1:]


# ------------------------------------------------------------
# EVALUATE A COMPLETED PATH
# ------------------------------------------------------------

# CALCULATE TOTAL RADIATION EXPOSURE AND TRAVEL TIME.
#
# EXPOSURE IS THE SUM OF THE COSTS OF ALL VISITED CELLS.
# REVISITING A CELL ADDS ITS COST AGAIN.
# THE STARTING CELL IS INCLUDED ONCE.
#
# TRAVEL TIME IS THE NUMBER OF MOVEMENTS MADE.
# EACH MOVEMENT TAKES ONE TIME UNIT.
#
# THIS FUNCTION ALSO CHECKS THAT THE PATH DOES NOT
# ENTER OBSTACLES OR MAKE INVALID MOVEMENTS.

def evaluate_path(grid, path):

    if not path:
        raise ValueError("Path cannot be empty")

    for i, (row, col) in enumerate(path):

        if not is_valid_move(grid, row, col):
            raise ValueError(
                f"Blocked or out-of-bounds cell: {(row, col)}"
            )

        if i > 0:

            previous_row, previous_col = path[i - 1]

            distance = (
                abs(row - previous_row) +
                abs(col - previous_col)
            )

            if distance != 1:
                raise ValueError(
                    "Path contains a non-adjacent movement"
                )

    exposure = sum(grid[row][col] for row, col in path)

    travel_time = len(path) - 1

    return exposure, travel_time


# ------------------------------------------------------------
# SAVE AND LOAD GRIDS
# ------------------------------------------------------------

# SAVE THE GENERATED GRID TO A JSON FILE.
# RUNNING THIS AGAIN WITH THE SAME FILENAME
# OVERWRITES THE PREVIOUS SAVED GRID.

def save_grid(grid, filename="shared_grid.json"):

    with open(filename, "w") as file:
        json.dump(grid, file, indent=2)


# LOAD A PREVIOUSLY SAVED GRID.
# THIS ALLOWS EVERY GROUP MEMBER TO USE
# THE EXACT SAME MAP FOR ALGORITHM COMPARISONS.

def load_grid(filename="shared_grid.json"):

    with open(filename) as file:
        return json.load(file)


# ------------------------------------------------------------
# MAIN PROGRAM
# ------------------------------------------------------------

# THIS SECTION ONLY RUNS WHEN map_utils.py IS EXECUTED DIRECTLY.
# IT DOES NOT RUN WHEN ANOTHER FILE IMPORTS map_utils.
#
# GENERATE THE MAP, DISPLAY IT, SELECT START AND GOALS,
# AND SAVE THE GRID FOR THE OTHER ALGORITHMS.

if __name__ == "__main__":

    grid = create_grid(MAP_SIZE)

    print("Generated Radiation Map:")
    print("Map Size:", MAP_SIZE, "x", MAP_SIZE)

    for row in grid:
        print(
            " ".join(
                " XX" if value == -1 else f"{value:3}"
                for value in row
            )
        )

    start, goals = choose_positions(grid)

    print("\nStarting Position:", start)
    print("Goals:", goals)

    save_grid(grid)

    print("\nSaved shared_grid.json")
