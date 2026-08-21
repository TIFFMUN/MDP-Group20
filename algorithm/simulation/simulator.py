import os
import json
import random
import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List

from algos.maze_solver import MazeSolver
from entities.obstacle import Obstacle
from entities.cell_state import CellState
from motion.directions import Direction


class Simulator:
    """Run the algorithm without physical hardware and visualise the result."""

    _DEBUG_FILE = os.path.join(os.path.dirname(__file__), "../debug/obstacles.json")

    def __init__(
        self,
        solver: MazeSolver = None,
        grid_width: int = 20,
        grid_height: int = 20,
        robot_x: int = 1,
        robot_y: int = 1,
        robot_facing: Direction = Direction.NORTH,
    ):
        self.solver = solver or MazeSolver(
            grid_width=grid_width,
            grid_height=grid_height,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_facing=robot_facing,
        )
        self._debug_enabled = False
        self._debug_slot = 0

    # ------------------------------------------------------------------
    # Obstacle helpers
    # ------------------------------------------------------------------

    def add_obstacles(self, obstacles: list):
        """obstacles: list of (x, y, Direction, id) tuples."""
        for obs in obstacles:
            self.solver.add_obstacle(*obs)
        if self._debug_enabled:
            self._save_obstacles(obstacles)

    def generate_random_obstacles(self, count: int) -> list:
        gx = self.solver.grid.size_x
        gy = self.solver.grid.size_y
        padding = 5
        start = self.solver.robot.get_start_state()
        existing = self.solver.grid.obstacles
        existing_ids = [o.obs_id for o in existing]
        next_id = max(existing_ids, default=0) + 1

        forbidden = self._forbidden_area([(start.x, start.y)], padding=padding)
        placed = []

        for i in range(count):
            rx = random.randint(padding, gx - 1 - padding)
            ry = random.randint(padding, gy - 1 - padding)
            chosen_facing = None

            while (rx, ry) in forbidden or chosen_facing is None:
                rx = random.randint(padding, gx - 1 - padding)
                ry = random.randint(padding, gy - 1 - padding)
                if (rx, ry) in forbidden:
                    continue
                chosen_facing = self._smart_facing(rx, ry, existing)

            forbidden.update(self._forbidden_area([(rx, ry)]))
            obs_id = next_id + i
            placed.append((rx, ry, chosen_facing, obs_id))
            self.solver.add_obstacle(rx, ry, chosen_facing, obs_id)

        if self._debug_enabled:
            self._save_obstacles(placed)
        return placed

    def clear_obstacles(self):
        self.solver.clear_obstacles()

    # ------------------------------------------------------------------
    # Solving
    # ------------------------------------------------------------------

    def run(self):
        print("Calculating optimal path...")
        return self.solver.get_optimal_path()

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def animate(self, path: List[CellState], output_filename: str = "optimal_path.gif"):
        """Save an animated GIF of the robot following path."""
        start_state = self.solver.robot.get_start_state()
        obstacles = self.solver.grid.obstacles

        markers = {"^": [], ">": [], "v": [], "<": []}
        for obs in obstacles:
            sym = self._dir_symbol(obs.facing)
            markers[sym].append((obs.x, obs.y))

        trail_by_dir = {sym: [] for sym in markers}
        arc_by_angle = {45: [], 135: [], 225: [], 315: []}
        screenshots = []

        tick = 0
        prev = start_state
        num_arc_samples = 3

        for cell in path:
            dx = abs(prev.x - cell.x)
            dy = abs(prev.y - cell.y)
            if dx > 1 or dy > 1:
                arc_angle = self._turn_angle(prev.facing, cell.facing)
                if arc_angle == 0:
                    arc_angle = self._half_turn_angle(prev, cell)
                if arc_angle and arc_angle in arc_by_angle:
                    for k in range(1, num_arc_samples + 1):
                        arc_x = prev.x + k * (cell.x - prev.x) / (num_arc_samples + 1)
                        arc_y = prev.y + k * (cell.y - prev.y) / (num_arc_samples + 1)
                        arc_by_angle[arc_angle].append((arc_x, arc_y, tick))
                        tick += 1

            if cell.capture_ids:
                screenshots.append((cell.x, cell.y, tick))

            sym = self._dir_symbol(cell.facing)
            trail_by_dir[sym].append((cell.x, cell.y, tick))
            prev = cell
            tick += 1

        fig, ax = plt.subplots()
        ax.set(xlim=(0, 20), ylim=(0, 20))

        def update(frame):
            ax.clear()
            ax.set(xlim=(0, 20), ylim=(0, 20))
            ax.set_xticks(range(21))
            ax.set_yticks(range(21))
            ax.grid()

            ax.scatter(start_state.x, start_state.y,
                       marker=self._dir_symbol(start_state.facing),
                       color="red", s=100)

            for sym, pts in markers.items():
                if pts:
                    xs, ys = zip(*pts)
                    ax.scatter(xs, ys, marker=sym, color="black", s=300)

            for sym, trail in trail_by_dir.items():
                xs = [x for x, y, t in trail if frame - 2 <= t <= frame]
                ys = [y for x, y, t in trail if frame - 2 <= t <= frame]
                if xs:
                    ax.scatter(xs, ys, marker=sym, color="blue", s=80)

            angle_markers = {45: (3, 0, 45), 135: (3, 0, 135), 225: (3, 0, 225), 315: (3, 0, 315)}
            for angle, pts in arc_by_angle.items():
                xs = [x for x, y, t in pts if frame - 2 <= t <= frame]
                ys = [y for x, y, t in pts if frame - 2 <= t <= frame]
                if xs:
                    ax.scatter(xs, ys, marker=angle_markers[angle], color="green", s=120)

            snap_xs = [x for x, y, t in screenshots if frame > t]
            snap_ys = [y for x, y, t in screenshots if frame > t]
            if snap_xs:
                ax.scatter(snap_xs, snap_ys, color="green", s=100, marker="*")

        ani = animation.FuncAnimation(fig, update, frames=tick + 5, interval=300)
        out_dir = os.path.join(os.path.dirname(__file__), "../animations")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.realpath(os.path.join(out_dir, output_filename))
        print(f"Saving animation to {out_path}")
        ani.save(out_path, writer="pillow")

    # ------------------------------------------------------------------
    # Debug persistence
    # ------------------------------------------------------------------

    def enable_debug(self, save_slot: int = 0):
        self._debug_enabled = True
        self._debug_slot = save_slot
        os.makedirs(os.path.dirname(self._DEBUG_FILE), exist_ok=True)
        if not os.path.exists(self._DEBUG_FILE):
            with open(self._DEBUG_FILE, "w") as f:
                json.dump({"save_1": [], "save_2": [], "save_3": [], "last": []}, f)

    def disable_debug(self):
        self._debug_enabled = False
        self._debug_slot = 0

    def load_obstacles(self, slot: int = 0):
        try:
            with open(self._DEBUG_FILE) as f:
                data = json.load(f)
        except IOError:
            return []

        key = "last" if slot == 0 else str(slot)
        for entry in data.get(key, []):
            self.solver.add_obstacle(entry["x"], entry["y"], entry["direction"], entry["id"])
        return data.get(key, [])

    def _save_obstacles(self, obstacles: list):
        try:
            with open(self._DEBUG_FILE) as f:
                data = json.load(f)
        except IOError:
            data = {"save_1": [], "save_2": [], "save_3": [], "last": []}

        serialised = [{"x": o[0], "y": o[1], "direction": o[2], "id": o[3]} for o in obstacles]
        data["last"] = serialised
        if self._debug_slot in (1, 2, 3):
            data[str(self._debug_slot)] = serialised

        with open(self._DEBUG_FILE, "w") as f:
            json.dump(data, f, indent=4)

    # ------------------------------------------------------------------
    # Static helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _dir_symbol(facing: Direction) -> str:
        return {Direction.NORTH: "^", Direction.EAST: ">",
                Direction.SOUTH: "v", Direction.WEST: "<"}.get(facing, "o")

    @staticmethod
    def _turn_angle(d1: Direction, d2: Direction):
        table = {
            (Direction.NORTH, Direction.EAST): 315,
            (Direction.EAST,  Direction.SOUTH): 225,
            (Direction.SOUTH, Direction.WEST): 135,
            (Direction.WEST,  Direction.NORTH): 45,
            (Direction.NORTH, Direction.WEST):  45,
            (Direction.WEST,  Direction.SOUTH): 135,
            (Direction.SOUTH, Direction.EAST):  225,
            (Direction.EAST,  Direction.NORTH): 315,
        }
        if d1 == d2:
            return 0
        return table.get((d1, d2))

    @staticmethod
    def _half_turn_angle(prev: CellState, nxt: CellState):
        d = prev.facing
        dx = nxt.x - prev.x
        dy = nxt.y - prev.y
        if d == Direction.NORTH:
            return 315 if dx > 0 and dy > 0 else (45 if dx < 0 and dy > 0 else (225 if dx > 0 else 135))
        if d == Direction.EAST:
            return 225 if dy > 0 and dx > 0 else (315 if dy < 0 and dx > 0 else (135 if dy > 0 else 45))
        if d == Direction.SOUTH:
            return 135 if dx > 0 and dy < 0 else (225 if dx < 0 and dy < 0 else (45 if dx > 0 else 315))
        if d == Direction.WEST:
            return 45 if dy > 0 and dx > 0 else (135 if dy < 0 and dx > 0 else (315 if dy > 0 else 225))
        return None

    @staticmethod
    def _forbidden_area(positions, padding: int = 2) -> set:
        forbidden = set()
        for x, y in positions:
            for di in range(-padding, padding + 1):
                for dj in range(-padding, padding + 1):
                    forbidden.add((x + di, y + dj))
        return forbidden

    def _smart_facing(self, x, y, existing_obstacles) -> Direction:
        available = []
        for d in (Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST):
            if not self._approach_blocked(x, y, d, existing_obstacles):
                if not self._too_close_to_edge(x, y, d):
                    available.append(d)
        return random.choice(available) if available else None

    def _approach_blocked(self, x, y, facing, obstacles, padding=5):
        half = padding // 2
        for obs in obstacles:
            if facing == Direction.NORTH and x - half < obs.x < x + half and y < obs.y < y + padding:
                return True
            if facing == Direction.EAST and y - half < obs.y < y + half and x < obs.x < x + padding:
                return True
            if facing == Direction.SOUTH and x - half < obs.x < x + half and y - padding < obs.y < y:
                return True
            if facing == Direction.WEST and y - half < obs.y < y + half and x - padding < obs.x < x:
                return True
        return False

    def _too_close_to_edge(self, x, y, facing, padding=6):
        gx = self.solver.grid.size_x
        gy = self.solver.grid.size_y
        return (
            (facing == Direction.NORTH and y > gy - padding) or
            (facing == Direction.EAST  and x > gx - padding) or
            (facing == Direction.SOUTH and y < padding) or
            (facing == Direction.WEST  and x < padding)
        )
