import heapq
import math

from entities.cell_state import CellState
from entities.grid import Grid
from motion.directions import Direction
from motion.motions import Motion
from config import (
    STRAIGHT_MOVE_VECTORS,
    FULL_TURN_PENALTY,
    HALF_TURN_PENALTY,
    REVERSE_PENALTY,
    OBSTACLE_PROXIMITY_COST,
    FULL_TURN_STEPS,
    HALF_TURN_STEPS,
)


class AStarSearch:
    """
    Stateful A* search over (x, y, facing) space.
    Caches path and cost results so repeated queries reuse prior work.
    """

    def __init__(self, grid: Grid):
        self.grid = grid
        self._path_cache: dict = {}
        self._cost_cache: dict = {}
        self._motion_cache: dict = {}

    def search(self, start: CellState, goal: CellState) -> None:
        """Run A* from start to goal and cache the result. No-op if already cached."""
        if (start, goal) in self._path_cache:
            return

        g_scores = {(start.x, start.y, start.facing): 0}
        heap = [(self._heuristic(start, goal), start.x, start.y, start.facing)]
        visited = set()
        parent = {}

        while heap:
            _, cx, cy, cf = heapq.heappop(heap)

            if (cx, cy, cf) in visited:
                continue

            if goal.matches(cx, cy, cf):
                self._store_path(start, goal, parent, g_scores[(cx, cy, cf)])
                return

            visited.add((cx, cy, cf))
            g = g_scores[(cx, cy, cf)]

            for nx, ny, nf, proximity_cost, motion in self._neighbors(cx, cy, cf):
                if (nx, ny, nf) in visited:
                    continue

                self._cache_motion(cx, cy, cf, nx, ny, nf, motion)

                step_cost = self._step_cost(cf, nf, motion, proximity_cost)
                capture_penalty = goal.position_penalty if goal.matches(nx, ny, nf) else 0
                new_g = g + step_cost + capture_penalty

                if (nx, ny, nf) not in g_scores or g_scores[(nx, ny, nf)] > new_g:
                    g_scores[(nx, ny, nf)] = new_g
                    f = new_g + self._heuristic(CellState(nx, ny, nf), goal)
                    heapq.heappush(heap, (f, nx, ny, nf))
                    parent[(nx, ny, nf)] = (cx, cy, cf)

    def get_path(self, start: CellState, goal: CellState):
        return self._path_cache.get((start, goal))

    def get_cost(self, start: CellState, goal: CellState):
        return self._cost_cache.get((start, goal))

    def get_motion(self, fx, fy, ff, tx, ty, tf):
        fwd_key = (fx, fy, ff, tx, ty, tf)
        rev_key = (tx, ty, tf, fx, fy, ff)
        if fwd_key in self._motion_cache:
            return self._motion_cache[fwd_key]
        if rev_key in self._motion_cache:
            return self._motion_cache[rev_key].opposite()
        return None

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _neighbors(self, x, y, facing):
        neighbors = []

        for dx, dy, move_dir in STRAIGHT_MOVE_VECTORS:
            if move_dir != facing:
                continue

            # straight forward
            if self.grid.is_straight_reachable(x + dx, y + dy):
                prox = self._proximity_cost(x + dx, y + dy)
                neighbors.append((x + dx, y + dy, move_dir, prox, Motion.FORWARD))

            # straight reverse
            if self.grid.is_straight_reachable(x - dx, y - dy):
                prox = self._proximity_cost(x - dx, y - dy)
                neighbors.append((x - dx, y - dy, move_dir, prox, Motion.REVERSE))

            # half-turn offsets
            hdx, hdy = self._half_turn_delta(facing)
            half_moves = self._half_turn_candidates(x, y, dx, dy, hdx, hdy, facing)
            neighbors.extend(half_moves)

        # full turns
        neighbors.extend(self._full_turn_candidates(x, y, facing))
        return neighbors

    def _half_turn_candidates(self, x, y, dx, dy, hdx, hdy, facing):
        candidates = []
        is_ns = facing in (Direction.NORTH, Direction.SOUTH)

        pairs = [
            (x + hdx, y + hdy, Motion.FORWARD_OFFSET_RIGHT),
            (x - hdx, y + hdy, Motion.FORWARD_OFFSET_LEFT),
            (x + hdx, y - hdy, Motion.REVERSE_OFFSET_RIGHT),
            (x - hdx, y - hdy, Motion.REVERSE_OFFSET_LEFT),
        ] if is_ns else [
            (x + hdx, y - hdy, Motion.FORWARD_OFFSET_RIGHT),
            (x + hdx, y + hdy, Motion.FORWARD_OFFSET_LEFT),
            (x - hdx, y - hdy, Motion.REVERSE_OFFSET_RIGHT),
            (x - hdx, y + hdy, Motion.REVERSE_OFFSET_LEFT),
        ]

        for nx, ny, motion in pairs:
            if self.grid.is_half_turn_reachable(x, y, nx, ny):
                prox = self._proximity_cost(nx, ny)
                candidates.append((nx, ny, facing, prox, motion))
        return candidates

    def _full_turn_candidates(self, x, y, facing):
        big, small = FULL_TURN_STEPS
        candidates = []

        turn_map = {
            (Direction.NORTH, Direction.EAST):  [(x + big, y + small, Motion.FORWARD_RIGHT_TURN), (x - small, y - big, Motion.REVERSE_LEFT_TURN)],
            (Direction.EAST,  Direction.NORTH):  [(x + small, y + big, Motion.FORWARD_LEFT_TURN),  (x - big, y - small, Motion.REVERSE_RIGHT_TURN)],
            (Direction.EAST,  Direction.SOUTH):  [(x + small, y - big, Motion.FORWARD_RIGHT_TURN), (x - big, y + small, Motion.REVERSE_LEFT_TURN)],
            (Direction.SOUTH, Direction.EAST):   [(x + big, y - small, Motion.FORWARD_LEFT_TURN),  (x - small, y + big, Motion.REVERSE_RIGHT_TURN)],
            (Direction.SOUTH, Direction.WEST):   [(x - big, y - small, Motion.FORWARD_RIGHT_TURN), (x + small, y + big, Motion.REVERSE_LEFT_TURN)],
            (Direction.WEST,  Direction.SOUTH):  [(x - small, y - big, Motion.FORWARD_LEFT_TURN),  (x + big, y + small, Motion.REVERSE_RIGHT_TURN)],
            (Direction.WEST,  Direction.NORTH):  [(x - small, y + big, Motion.FORWARD_RIGHT_TURN), (x + big, y - small, Motion.REVERSE_LEFT_TURN)],
            (Direction.NORTH, Direction.WEST):   [(x - big, y + small, Motion.FORWARD_LEFT_TURN),  (x + small, y - big, Motion.REVERSE_RIGHT_TURN)],
        }

        for (from_dir, to_dir), moves in turn_map.items():
            if facing != from_dir:
                continue
            for nx, ny, motion in moves:
                if self.grid.is_turn_reachable(x, y, nx, ny, facing):
                    prox = self._proximity_cost(nx, ny)
                    candidates.append((nx, ny, to_dir, prox + 10, motion))

        return candidates

    def _proximity_cost(self, x, y) -> int:
        padding = 2
        for obs in self.grid.obstacles:
            if abs(obs.x - x) <= padding and abs(obs.y - y) <= padding:
                return OBSTACLE_PROXIMITY_COST
        return 0

    @staticmethod
    def _step_cost(from_facing, to_facing, motion: Motion, proximity_cost: int) -> int:
        rot_cost = FULL_TURN_PENALTY * Direction.turn_cost(from_facing, to_facing)
        rot_cost = max(rot_cost, 1)
        rev_cost = max(REVERSE_PENALTY * int(motion.is_reverse()), 1)
        half_cost = max(HALF_TURN_PENALTY * int(motion.is_half_turn()), 1)
        return rot_cost * rev_cost * half_cost + proximity_cost

    @staticmethod
    def _heuristic(state: CellState, goal: CellState) -> int:
        return abs(state.x - goal.x) + abs(state.y - goal.y)

    def _store_path(self, start: CellState, goal: CellState, parent: dict, cost: int):
        self._cost_cache[(start, goal)] = cost
        self._cost_cache[(goal, start)] = cost

        path = []
        node = (goal.x, goal.y, goal.facing)
        while node in parent:
            path.append(node)
            node = parent[node]
        path.append(node)

        self._path_cache[(start, goal)] = path[::-1]
        self._path_cache[(goal, start)] = path

    def _cache_motion(self, fx, fy, ff, tx, ty, tf, motion):
        fwd = (fx, fy, ff, tx, ty, tf)
        rev = (tx, ty, tf, fx, fy, ff)
        if fwd not in self._motion_cache and rev not in self._motion_cache:
            self._motion_cache[fwd] = motion

    @staticmethod
    def _half_turn_delta(facing: Direction):
        deltas = {
            Direction.NORTH: (HALF_TURN_STEPS[1], HALF_TURN_STEPS[0]),
            Direction.SOUTH: (-HALF_TURN_STEPS[1], -HALF_TURN_STEPS[0]),
            Direction.EAST:  (HALF_TURN_STEPS[0], HALF_TURN_STEPS[1]),
            Direction.WEST:  (-HALF_TURN_STEPS[0], -HALF_TURN_STEPS[1]),
        }
        if facing not in deltas:
            raise ValueError(f"Invalid direction {facing}")
        return deltas[facing]
