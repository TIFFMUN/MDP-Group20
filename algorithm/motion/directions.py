from enum import Enum


class Direction(int, Enum):
    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3
    SKIP = 4

    def __int__(self):
        return self.value

    @staticmethod
    def turn_cost(from_dir, to_dir):
        """Cost of rotating from one cardinal direction to another (0 = no turn, 1 = 90-degree turn)."""
        if from_dir in (Direction.NORTH, Direction.SOUTH):
            if to_dir in (Direction.EAST, Direction.WEST):
                return 1
            if to_dir == from_dir:
                return 0
            raise ValueError(f"Cannot turn 180 degrees from {from_dir}")
        if from_dir in (Direction.EAST, Direction.WEST):
            if to_dir in (Direction.NORTH, Direction.SOUTH):
                return 1
            if to_dir == from_dir:
                return 0
            raise ValueError(f"Cannot turn 180 degrees from {from_dir}")
        raise ValueError(f"Invalid direction {from_dir}")

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name
