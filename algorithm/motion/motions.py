from enum import Enum


class Motion(int, Enum):
    """
    10 movement primitives for the robot.
    Symmetric design: value + opposite_value == 10 for all non-CAPTURE motions.
    """
    FORWARD_LEFT_TURN = 0
    FORWARD_OFFSET_LEFT = 1
    FORWARD = 2
    FORWARD_OFFSET_RIGHT = 3
    FORWARD_RIGHT_TURN = 4

    REVERSE_RIGHT_TURN = 6
    REVERSE_OFFSET_LEFT = 7
    REVERSE = 8
    REVERSE_OFFSET_RIGHT = 9
    REVERSE_LEFT_TURN = 10

    CAPTURE = 1000

    def __int__(self):
        return self.value

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.name

    def __eq__(self, other: "Motion"):
        return self.value == other.value

    def opposite(self):
        if self == Motion.CAPTURE:
            return Motion.CAPTURE
        opp_val = 10 - self.value
        if opp_val == 5 or not (0 <= opp_val <= 10):
            raise ValueError(f"No opposite for motion {self}")
        return Motion(opp_val)

    def is_combinable(self):
        """Only straight forward/reverse moves can be merged into a single longer command."""
        return self.value in (2, 8)

    def is_reverse(self):
        return self in (
            Motion.REVERSE,
            Motion.REVERSE_OFFSET_LEFT,
            Motion.REVERSE_OFFSET_RIGHT,
            Motion.REVERSE_LEFT_TURN,
            Motion.REVERSE_RIGHT_TURN,
        )

    def is_half_turn(self):
        return self in (
            Motion.FORWARD_OFFSET_LEFT,
            Motion.FORWARD_OFFSET_RIGHT,
            Motion.REVERSE_OFFSET_LEFT,
            Motion.REVERSE_OFFSET_RIGHT,
        )
