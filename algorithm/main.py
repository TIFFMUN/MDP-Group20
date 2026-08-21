from flask import Flask, request, jsonify

from algos.maze_solver import MazeSolver
from motion.directions import Direction
from motion.command_gen import CommandGenerator

app = Flask(__name__)


@app.route("/path", methods=["POST"])
def compute_path():
    data = request.get_json()

    solver = MazeSolver(
        grid_width=data.get("grid_width", 20),
        grid_height=data.get("grid_height", 20),
        robot_x=data.get("robot_x", 1),
        robot_y=data.get("robot_y", 1),
        robot_facing=Direction(data.get("robot_facing", Direction.NORTH)),
    )

    for obs in data.get("obstacles", []):
        solver.add_obstacle(obs["x"], obs["y"], Direction(obs["d"]), obs["id"])

    path, cost = solver.get_optimal_path()
    motions, obs_ids = solver.path_to_motions(path)

    gen = CommandGenerator()
    commands = gen.generate_commands(motions, obs_ids)

    return jsonify({
        "path": [cell.to_dict() for cell in path],
        "commands": commands,
        "cost": cost,
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
