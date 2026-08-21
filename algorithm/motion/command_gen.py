from motion.motions import Motion


class CommandGenerator:
    """Converts a sequence of Motion values into STM command strings."""

    SEP = "|"
    FIN = "FIN"
    UNIT_DIST = 10

    # Tunable turn angles (degrees) — calibrate against your robot
    FWD_TURN_ANGLE = 25
    REV_TURN_ANGLE = 25
    FWD_RIGHT_FINAL_ANGLE = 86
    FWD_LEFT_FINAL_ANGLE = 87
    REV_RIGHT_FINAL_ANGLE = 89
    REV_LEFT_FINAL_ANGLE = 88

    def __init__(self, straight_speed: int = 50, turn_speed: int = 50):
        self.straight_speed = straight_speed
        self.turn_speed = turn_speed

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate_commands(self, motions: list, obstacle_ids: list) -> list:
        if not motions:
            return []

        raw_commands = self._motions_to_raw_commands(motions, obstacle_ids)
        return self._merge_consecutive_straights(raw_commands)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _motions_to_raw_commands(self, motions: list, obstacle_ids: list) -> list:
        commands = []
        snap_idx = 0
        run_length = 1
        prev = motions[0]

        for cur in motions[1:]:
            if cur == prev and cur.is_combinable():
                run_length += 1
            else:
                if prev == Motion.CAPTURE:
                    commands += [f"M0|0|0", f"SNAP{obstacle_ids[snap_idx]}"]
                    snap_idx += 1
                else:
                    commands += self._single_motion_to_cmds(prev, run_length)
                run_length = 1
            prev = cur

        if prev == Motion.CAPTURE:
            commands += [f"M0|0|0", f"SNAP{obstacle_ids[snap_idx]}"]
        else:
            commands += self._single_motion_to_cmds(prev, run_length)

        commands.append(self.FIN)
        return commands

    def _single_motion_to_cmds(self, motion: Motion, count: int = 1) -> list:
        dist = count * self.UNIT_DIST
        ss, ts = self.straight_speed, self.turn_speed
        S = self.SEP

        if motion == Motion.FORWARD:
            return [f"T{ss}{S}0{S}{dist}"]
        if motion == Motion.REVERSE:
            return [f"t{ss}{S}0{S}{dist}"]

        if motion == Motion.FORWARD_LEFT_TURN:
            return [
                f"T{ts}{S}-{self.FWD_TURN_ANGLE}{S}{self.FWD_LEFT_FINAL_ANGLE}",
                f"T{ss}{S}0{S}6",
            ]
        if motion == Motion.FORWARD_RIGHT_TURN:
            return [
                f"T{ss}{S}0{S}5",
                f"T{ts}{S}{self.FWD_TURN_ANGLE}{S}{self.FWD_RIGHT_FINAL_ANGLE}",
                f"T{ss}{S}0{S}12",
            ]
        if motion == Motion.REVERSE_LEFT_TURN:
            return [
                f"t{ss}{S}0{S}6",
                f"t{ts}{S}-{self.REV_TURN_ANGLE}{S}{self.REV_LEFT_FINAL_ANGLE}",
            ]
        if motion == Motion.REVERSE_RIGHT_TURN:
            return [
                f"t{ss}{S}0{S}7",
                f"t{ts}{S}{self.REV_TURN_ANGLE}{S}{self.REV_RIGHT_FINAL_ANGLE}",
                f"t{ss}{S}0{S}4",
            ]

        if motion == Motion.FORWARD_OFFSET_LEFT:
            return [
                f"T{ts}{S}-14{S}21",
                f"T{ts}{S}17{S}21",
            ]
        if motion == Motion.FORWARD_OFFSET_RIGHT:
            return [
                f"T{ts}{S}14{S}21",
                f"T{ss}{S}-17{S}21",
            ]
        if motion == Motion.REVERSE_OFFSET_LEFT:
            return [
                f"t{ts}{S}-15{S}24",
                f"t{ts}{S}25{S}25",
            ]
        if motion == Motion.REVERSE_OFFSET_RIGHT:
            return [
                f"t{ts}{S}15{S}24",
                f"t{ts}{S}-25{S}25",
            ]

        raise ValueError(f"Unhandled motion: {motion}")

    @staticmethod
    def _merge_consecutive_straights(commands: list) -> list:
        """Merge adjacent forward-only or reverse-only straight commands."""
        merged = []
        pending = None

        for cmd in commands:
            if cmd in (CommandGenerator.FIN,) or cmd.startswith("SNAP") or cmd == "M0|0|0":
                if pending:
                    merged.append(pending)
                    pending = None
                merged.append(cmd)
                continue

            if pending is None:
                pending = cmd
                continue

            combined = CommandGenerator._try_merge(pending, cmd)
            if combined:
                pending = combined
            else:
                merged.append(pending)
                pending = cmd

        return merged

    @staticmethod
    def _try_merge(cmd_a: str, cmd_b: str) -> str | None:
        """Return merged command string if both are zero-angle straights, else None."""
        parts_a = cmd_a.split("|")
        parts_b = cmd_b.split("|")

        angle_a, angle_b = int(parts_a[1]), int(parts_b[1])
        if angle_a != 0 or angle_b != 0:
            return None

        dist_a, dist_b = int(parts_a[2]), int(parts_b[2])
        dir_a, dir_b = parts_a[0][0], parts_b[0][0]  # 'T' or 't'
        speed = parts_a[0][1:]

        if dir_a == dir_b:
            return f"{dir_a}{speed}|0|{dist_a + dist_b}"

        # opposite directions: net movement
        if dist_a > dist_b:
            return f"{dir_a}{speed}|0|{dist_a - dist_b}"
        return f"{dir_b}{speed}|0|{dist_b - dist_a}"
