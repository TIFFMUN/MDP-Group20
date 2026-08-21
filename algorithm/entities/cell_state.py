from motion.directions import Direction
from config import (
    COLLISION_BUFFER,
    CAPTURE_POSITION_COST,
    NEAR_OBSTACLE_COST,
)
from typing import List
import math


class CellState:
    """A node in the search space: (x, y, facing direction) with optional capture metadata."""

    def __init__(
        self,
        x: int,
        y: int,
        facing: Direction = Direction.NORTH,
        capture_ids: List = None,
        position_penalty: int = 0,
    ):
        self.x = x
        self.y = y
        self.facing = facing
        self.capture_ids = capture_ids if capture_ids is not None else []
        self.position_penalty = position_penalty

    def __repr__(self):
        return f"CellState(x={self.x}, y={self.y}, facing={self.facing}, captures={self.capture_ids})"

    def same_position(self, x: int, y: int) -> bool:
        return self.x == x and self.y == y

    def matches(self, x: int, y: int, facing: Direction) -> bool:
        return self.x == x and self.y == y and self.facing == facing

    def add_capture(self, capture_id: str):
        self.capture_ids.append(capture_id)

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "d": self.facing, "s": self.capture_ids}
