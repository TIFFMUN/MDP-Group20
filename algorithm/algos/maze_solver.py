from entities.cell_state import CellState
from entities.grid import Grid
from entities.obstacle import Obstacle
from entities.robot import Robot
from motion.directions import Direction
from motion.motions import Motion
from algos.astar import AStarSearch
from algos.tsp import TSPSolver


class MazeSolver:
    """
    Top-level orchestrator: sets up the grid, runs A* + TSP,
    and converts the resulting path into motion primitives.
    """

    def __init__(
        self,
        grid_width: int = 20,
        grid_height: int = 20,
        robot: Robot = None,
        robot_x: int = 1,
        robot_y: int = 1,
        robot_facing: Direction = Direction.NORTH,
    ):
        self.grid = Grid(grid_width, grid_height)
        self.robot = robot if robot else Robot(robot_x, robot_y, robot_facing)
        self._searcher = AStarSearch(self.grid)
        self._tsp = TSPSolver(self._searcher)

    # ------------------------------------------------------------------
    # Grid mutation
    # ------------------------------------------------------------------

    def add_obstacle(self, x: int, y: int, image_facing: Direction, obs_id: int):
        self.grid.add_obstacle(Obstacle(x, y, image_facing, obs_id))

    def clear_obstacles(self):
        self.grid.clear_obstacles()

    # ------------------------------------------------------------------
    # Path finding
    # ------------------------------------------------------------------

    def get_optimal_path(self):
        """
        Returns:
            (path: List[CellState], cost: float)
        """
        viewpoint_groups = self.grid.get_all_capture_viewpoints()
        path, cost = self._tsp.solve(self.robot.get_start_state(), viewpoint_groups)
        path = self._annotate_captures(path, viewpoint_groups)
        return path, cost

    def path_to_motions(self, path):
        """
        Convert a CellState path to (motion_list, obstacle_id_list).
        """
        motions = []
        obs_ids = []

        for i in range(len(path) - 1):
            src = path[i]
            dst = path[i + 1]
            motion = self._searcher.get_motion(
                src.x, src.y, src.facing,
                dst.x, dst.y, dst.facing,
            )
            if motion is None:
                raise ValueError(f"No motion found between {src} and {dst}")
            motions.append(motion)

            if dst.capture_ids:
                for cid in dst.capture_ids:
                    motions.append(Motion.CAPTURE)
                    obs_ids.append(cid)

        return motions, obs_ids

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _annotate_captures(self, path, viewpoint_groups):
        """Add capture metadata to path nodes that correspond to obstacle viewpoints."""
        flat_viewpoints = [vp for group in viewpoint_groups for vp in group]

        for node in path:
            for vp in flat_viewpoints:
                if node.matches(vp.x, vp.y, vp.facing) and vp.capture_ids:
                    obs = self.grid.find_obstacle_by_id(vp.capture_ids[0] if isinstance(vp.capture_ids, list) else vp.capture_ids)
                    if obs:
                        rel_pos = self._capture_relative_position(node, obs)
                        node.add_capture(f"{obs.obs_id}_{rel_pos}")
        return path

    @staticmethod
    def _capture_relative_position(cell: CellState, obs: Obstacle) -> str:
        """Return 'L', 'R', or 'C' describing the obstacle's position relative to the robot camera."""
        x, y, facing = cell.x, cell.y, cell.facing
        ox, oy = obs.x, obs.y

        if facing == Direction.NORTH:
            if ox == x and oy > y:
                return "C"
            return "L" if ox < x else "R"
        if facing == Direction.SOUTH:
            if ox == x and oy < y:
                return "C"
            return "R" if ox < x else "L"
        if facing == Direction.EAST:
            if oy == y and ox > x:
                return "C"
            return "R" if oy < y else "L"
        if facing == Direction.WEST:
            if oy == y and ox < x:
                return "C"
            return "L" if oy < y else "R"
        raise ValueError(f"Invalid facing {facing}")
