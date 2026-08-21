from motion.directions import Direction

# A* cost hyperparameters
FULL_TURN_PENALTY = 6
HALF_TURN_PENALTY = 5 * 2
REVERSE_PENALTY = 3
OBSTACLE_PROXIMITY_COST = 1000
CAPTURE_POSITION_COST = 100
NEAR_OBSTACLE_COST = 50

# Collision detection padding
FULL_TURN_PADDING = 2
MID_TURN_PADDING = 2

# Turning geometry (grid units)
TURN_RADIUS = 1
FULL_TURN_STEPS = [5 * TURN_RADIUS, 3 * TURN_RADIUS]
HALF_TURN_STEPS = [4 * TURN_RADIUS, 1 * TURN_RADIUS]

# Straight movement vectors keyed by resulting direction
STRAIGHT_MOVE_VECTORS = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
]

# A* iteration cap
MAX_SEARCH_ITERATIONS = 2000

# Robot/obstacle collision buffer (cells)
COLLISION_BUFFER = 1
