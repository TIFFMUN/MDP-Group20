from entities.cell_state import CellState
from motion.directions import Direction
from typing import List


class Robot:
    """Tracks the robot's start state and historical path."""

    def __init__(self, start_x: int, start_y: int, start_facing: Direction):
        self.path: List[CellState] = [CellState(start_x, start_y, start_facing)]

    def get_start_state(self) -> CellState:
        return self.path[0]
