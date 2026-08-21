from motion.directions import Direction
from entities.cell_state import CellState
from entities.obstacle import Obstacle
from config import FULL_TURN_PADDING, MID_TURN_PADDING, COLLISION_BUFFER
from typing import List
import math


class Grid:
    """20×20 arena grid. Tracks obstacles and validates robot movements."""

    size_x: int = 20
    size_y: int = 20

    def __init__(self, size_x: int, size_y: int):
        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []

    # ------------------------------------------------------------------
    # Obstacle management
    # ------------------------------------------------------------------

    def add_obstacle(self, obstacle: Obstacle):
        if not any(obs == obstacle for obs in self.obstacles):
            self.obstacles.append(obstacle)

    def clear_obstacles(self):
        self.obstacles = []

    def get_obstacles(self) -> List[Obstacle]:
        return self.obstacles

    def find_obstacle_by_id(self, obs_id: int):
        return next((obs for obs in self.obstacles if obs.obs_id == obs_id), None)

    # ------------------------------------------------------------------
    # Validity / reachability checks
    # ------------------------------------------------------------------

    def is_valid_coord(self, x: int, y: int) -> bool:
        return 1 <= x < self.size_x - 1 and 1 <= y < self.size_y - 1

    def is_valid_cell_state(self, state: CellState) -> bool:
        return self.is_valid_coord(state.x, state.y)

    def is_straight_reachable(self, x: int, y: int) -> bool:
        """Check a straight-move destination is in bounds and not too close to any obstacle."""
        if not self.is_valid_coord(x, y):
            return False
        for obs in self.obstacles:
            if abs(obs.x - x) + abs(obs.y - y) <= 2:
                return False
            if max(abs(obs.x - x), abs(obs.y - y)) < 2:
                return False
        return True

    def is_turn_reachable(self, x: int, y: int, nx: int, ny: int, facing: Direction) -> bool:
        """Check the arc of a full turn is clear of all obstacles."""
        if not self.is_valid_coord(x, y) or not self.is_valid_coord(nx, ny):
            return False

        arc_samples = self._arc_sample_points(x, y, nx, ny, facing)

        for obs in self.obstacles:
            # pre-turn clearance
            if math.sqrt((obs.x - x) ** 2 + (obs.y - y) ** 2) < FULL_TURN_PADDING:
                return False
            # post-turn clearance
            if math.sqrt((obs.x - nx) ** 2 + (obs.y - ny) ** 2) < FULL_TURN_PADDING:
                return False
            # mid-arc clearance
            for px, py in arc_samples:
                if math.sqrt((obs.x - px) ** 2 + (obs.y - py) ** 2) < MID_TURN_PADDING:
                    return False
        return True

    def is_half_turn_reachable(self, x: int, y: int, nx: int, ny: int) -> bool:
        """Check a lateral offset move (half-turn) is clear of all obstacles."""
        if not self.is_valid_coord(x, y) or not self.is_valid_coord(nx, ny):
            return False

        padding = 2 * COLLISION_BUFFER
        lo_x, hi_x = (x, nx) if x <= nx else (nx, x)
        lo_y, hi_y = (y, ny) if y <= ny else (ny, y)

        for obs in self.obstacles:
            if (hi_x - lo_x) >= (hi_y - lo_y):
                # wider in x — pad y
                if lo_x <= obs.x <= hi_x and lo_y - padding <= obs.y <= hi_y + padding:
                    return False
            else:
                # wider in y — pad x
                if lo_x - padding <= obs.x <= hi_x + padding and lo_y <= obs.y <= hi_y:
                    return False
        return True

    # ------------------------------------------------------------------
    # Viewpoint generation
    # ------------------------------------------------------------------

    def get_all_capture_viewpoints(self) -> List[List[CellState]]:
        """Return one list of candidate viewpoints per non-SKIP obstacle."""
        result = []
        for obs in self.obstacles:
            if obs.facing == Direction.SKIP:
                continue
            valid_views = [
                vp for vp in obs.get_capture_viewpoints(self)
                if self.is_straight_reachable(vp.x, vp.y)
            ]
            result.append(valid_views)
        return result

    # ------------------------------------------------------------------
    # Class-level bounds check (used by Obstacle before Grid exists)
    # ------------------------------------------------------------------

    @classmethod
    def is_within_bounds(cls, x: int, y: int) -> bool:
        return 0 <= x < cls.size_x and 0 <= y < cls.size_y

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _arc_sample_points(x, y, nx, ny, facing: Direction):
        """
        Three sample points along the turn arc between (x,y) and (nx,ny).
        Uses the midpoint and the triangle corner to approximate the curve.
        """
        mid_x, mid_y = (x + nx) / 2, (y + ny) / 2

        if facing in (Direction.NORTH, Direction.SOUTH):
            corner_x, corner_y = x, ny
        else:
            corner_x, corner_y = nx, y

        return [
            ((x + mid_x) / 2, mid_y),
            ((corner_x + mid_x) / 2, (corner_y + mid_y) / 2),
            (mid_x, (ny + mid_y) / 2),
        ]
