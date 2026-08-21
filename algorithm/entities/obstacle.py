from motion.directions import Direction
from entities.cell_state import CellState
from config import COLLISION_BUFFER, CAPTURE_POSITION_COST, NEAR_OBSTACLE_COST
from typing import List


class Obstacle(CellState):
    """An obstacle on the grid with a facing direction (the side the image is on) and a unique id."""

    def __init__(self, x: int, y: int, image_facing: Direction, obs_id: int):
        super().__init__(x, y, image_facing)
        self.obs_id = obs_id

    def __eq__(self, other: "Obstacle") -> bool:
        return self.x == other.x and self.y == other.y and self.facing == other.facing

    def get_capture_viewpoints(self, grid) -> List[CellState]:
        """
        Return candidate CellStates from which the robot can photograph this obstacle.
        Positions form a T-shape in front of the image face.
        """
        offset = 2 * COLLISION_BUFFER
        facing = self.facing

        if facing == Direction.NORTH:
            candidate_positions = [
                (self.x, self.y + offset),
                (self.x - 1, self.y + 2 + offset),
                (self.x + 1, self.y + 2 + offset),
                (self.x, self.y + 1 + offset),
                (self.x, self.y + 2 + offset),
            ]
            robot_facing = Direction.SOUTH

        elif facing == Direction.SOUTH:
            candidate_positions = [
                (self.x, self.y - offset),
                (self.x + 1, self.y - 2 - offset),
                (self.x - 1, self.y - 2 - offset),
                (self.x, self.y - 1 - offset),
                (self.x, self.y - 2 - offset),
            ]
            robot_facing = Direction.NORTH

        elif facing == Direction.EAST:
            candidate_positions = [
                (self.x + offset, self.y),
                (self.x + 2 + offset, self.y + 1),
                (self.x + 2 + offset, self.y - 1),
                (self.x + 1 + offset, self.y),
                (self.x + 2 + offset, self.y),
            ]
            robot_facing = Direction.WEST

        else:  # WEST
            candidate_positions = [
                (self.x - offset, self.y),
                (self.x - 2 - offset, self.y + 1),
                (self.x - 2 - offset, self.y - 1),
                (self.x - 1 - offset, self.y),
                (self.x - 2 - offset, self.y),
            ]
            robot_facing = Direction.EAST

        penalties = [
            NEAR_OBSTACLE_COST,
            CAPTURE_POSITION_COST,
            CAPTURE_POSITION_COST,
            NEAR_OBSTACLE_COST // 2,
            0,
        ]

        viewpoints = []
        for pos, penalty in zip(candidate_positions, penalties):
            if grid.is_within_bounds(*pos):
                viewpoints.append(CellState(*pos, robot_facing, self.obs_id, penalty))
        return viewpoints
