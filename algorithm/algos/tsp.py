import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming

from entities.cell_state import CellState
from algos.astar import AStarSearch
from config import MAX_SEARCH_ITERATIONS


class TSPSolver:
    """
    Wraps the TSP dynamic-programming solver.
    Given a list of candidate viewpoint groups (one group per obstacle),
    finds the visit order that minimises total A* path cost.
    """

    def __init__(self, searcher: AStarSearch):
        self.searcher = searcher

    def solve(self, start: CellState, viewpoint_groups: list):
        """
        Args:
            start: Robot's starting CellState.
            viewpoint_groups: List[List[CellState]] — one inner list per obstacle.

        Returns:
            (optimal_path: List[CellState], total_cost: float)
        """
        min_cost = float("inf")
        best_path = []

        num_groups = len(viewpoint_groups)

        for visit_mask in self._visit_masks(num_groups):
            active_groups = []
            all_waypoints = [start]

            for i in range(num_groups):
                if visit_mask[i] == "1":
                    active_groups.append(viewpoint_groups[i])
                    all_waypoints.extend(viewpoint_groups[i])

            self._precompute_paths(all_waypoints)

            for selection in self._viewpoint_selections(active_groups):
                chosen_indices = [0]
                base = 1
                penalty = 0
                for group_idx, group in enumerate(active_groups):
                    chosen_indices.append(base + selection[group_idx])
                    penalty += group[selection[group_idx]].position_penalty
                    base += len(group)

                cost_matrix = self._build_cost_matrix(all_waypoints, chosen_indices)
                perm, dist = solve_tsp_dynamic_programming(cost_matrix)

                if dist + penalty >= min_cost:
                    continue

                min_cost = dist + penalty
                best_path = self._reconstruct_path(all_waypoints, chosen_indices, perm)

            if best_path:
                break

        return best_path, min_cost

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _precompute_paths(self, waypoints):
        for i in range(len(waypoints) - 1):
            for j in range(i + 1, len(waypoints)):
                self.searcher.search(waypoints[i], waypoints[j])

    def _build_cost_matrix(self, waypoints, chosen_indices):
        n = len(chosen_indices)
        matrix = np.zeros((n, n))

        for i in range(n - 1):
            for j in range(i + 1, n):
                s = waypoints[chosen_indices[i]]
                e = waypoints[chosen_indices[j]]
                cost = self.searcher.get_cost(s, e)
                matrix[i, j] = cost if cost is not None else 1e9
                matrix[j, i] = matrix[i, j]

        matrix[:, 0] = 0  # TSP: free return to depot (start)
        return matrix

    def _reconstruct_path(self, waypoints, chosen_indices, permutation):
        path = [waypoints[0]]
        for step in range(len(permutation) - 1):
            from_wp = waypoints[chosen_indices[permutation[step]]]
            to_wp = waypoints[chosen_indices[permutation[step + 1]]]
            segment = self.searcher.get_path(from_wp, to_wp)
            if segment:
                for node in segment[1:]:
                    path.append(CellState(node[0], node[1], node[2]))
        return path

    @staticmethod
    def _visit_masks(n: int):
        """Yield binary strings (all 1s first) representing which obstacles to include."""
        masks = [bin(i)[2:].zfill(n) for i in range(2 ** n)]
        masks.sort(key=lambda m: m.count("1"), reverse=True)
        return masks

    @staticmethod
    def _viewpoint_selections(groups, index=0, current=None, results=None, budget=MAX_SEARCH_ITERATIONS):
        """Enumerate all combinations of one viewpoint index per group."""
        if current is None:
            current = []
        if results is None:
            results = []

        if index == len(groups):
            results.append(current.copy())
            return results

        if budget == 0:
            return results

        budget -= 1
        for i in range(len(groups[index])):
            current.append(i)
            results = TSPSolver._viewpoint_selections(groups, index + 1, current, results, budget)
            current.pop()

        return results
