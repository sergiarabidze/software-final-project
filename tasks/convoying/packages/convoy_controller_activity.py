from tasks.convoying.packages.follow_types import (
    TargetInfo,
    ConvoyCommand,
    FAR,
    GOOD,
    CLOSE,
    TOO_CLOSE,
    LOST,
)


class ConvoyController:
    """
    Lane-following controls steering.
    Convoying controls speed via a PD controller on bottom_y position.

    The PD controller computes a smooth speed multiplier from how far
    the target's bottom_y deviates from a desired setpoint, eliminating
    the stop-go oscillation of fixed per-zone multipliers.

    Safety rules (unchanged from original):
    - TOO_CLOSE always stops immediately.
    - Lost while CLOSE: brief creep, then stop.
    - Lost while TOO_CLOSE: stop immediately.
    - Lost while FAR/GOOD: grace period at low speed.
    """

    def __init__(
            self,
            # Desired bottom_y as fraction of image height.
            # 0.43 = middle of the GOOD band (0.34–0.52).
            setpoint_ratio: float = 0.50,

            # P gain: higher = more aggressive response to distance error.
            kp: float = 2.0
            ,

            # D gain: dampens oscillation by resisting rapid error changes.
            # Keep low — too high suppresses startup movement.
            kd: float = 0.08,

            max_speed: float = 0.13,
            min_speed: float = 0.01,   # below this we treat output as zero

            lost_grace_frames: int = 35,
            lost_grace_multiplier: float = 0.25,
    ):
        self.setpoint_ratio        = setpoint_ratio
        self.kp                    = kp
        self.kd                    = kd
        self.max_speed             = max_speed
        self.min_speed             = min_speed
        self.lost_grace_frames     = lost_grace_frames
        self.lost_grace_multiplier = lost_grace_multiplier

        self._had_target           = False
        self._lost_frames          = 0
        self._last_distance_state  = LOST
        self._last_error           = None   # None = uninitialised; seeded on first frame

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def decide(
        self,
        target: TargetInfo,
        lane_left: float,
        lane_right: float,
        image_height: int = 240,
    ) -> ConvoyCommand:

        if target.found and target.distance_state != LOST:
            self._had_target = True
            self._lost_frames = 0
            self._last_distance_state = target.distance_state

            # Hard safety stop — never move into the truck.
            if target.distance_state == TOO_CLOSE:
                self._last_error = None   # reset so next acquisition is smooth
                return self._stop("target_too_close_stop")

            multiplier = self._pd_multiplier(target.bottom_y, image_height)
            reason = f"pd_following_{target.distance_state.lower()}"
            return self._scale(lane_left, lane_right, multiplier, reason)

        # ---- Target is lost ----
        self._lost_frames += 1
        self._last_error = None   # reset derivative on loss

        if self._last_distance_state == TOO_CLOSE:
            return self._stop("target_lost_after_too_close_emergency_stop")

        if self._last_distance_state == CLOSE and self._lost_frames <= 15:
            return self._scale(
                lane_left, lane_right, 0.25,
                "target_lost_after_close_creep_forward",
            )

        if self._last_distance_state == CLOSE:
            return self._stop("target_lost_after_close_stop")

        # FAR / GOOD — allow short grace through turn.
        if self._had_target and self._lost_frames <= self.lost_grace_frames:
            return self._scale(
                lane_left, lane_right, self.lost_grace_multiplier,
                "target_temporarily_lost_keep_lane_slowly",
            )

        return self._stop("target_lost_stop")

    # ------------------------------------------------------------------
    # PD controller
    # ------------------------------------------------------------------

    def _pd_multiplier(self, bottom_y: int, image_height: int) -> float:
        """
        Returns a [0, 1] speed multiplier.

        error > 0 → target below setpoint → too close → output 0 (slow/stop)
        error < 0 → target above setpoint → too far   → output > 0 (speed up)

        On the very first frame after acquisition, last_error is seeded to
        the current error so the derivative term is zero — avoids a large
        derivative kick suppressing the first movement.
        """
        if image_height <= 0:
            return 0.0

        current_ratio = bottom_y / float(image_height)
        error = current_ratio - self.setpoint_ratio

        # Seed on first acquisition to avoid derivative spike.
        if self._last_error is None:
            self._last_error = error

        derivative = error - self._last_error
        self._last_error = error

        raw = self.kp * (-error) + self.kd * (-derivative)
        return max(0.0, min(1.0, raw))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _scale(
        self,
        lane_left: float,
        lane_right: float,
        multiplier: float,
        reason: str,
    ) -> ConvoyCommand:
        left_speed  = self._clamp(lane_left  * multiplier)
        right_speed = self._clamp(lane_right * multiplier)
        moving = left_speed >= self.min_speed or right_speed >= self.min_speed

        return ConvoyCommand(
            should_move      = moving,
            left_speed       = left_speed  if moving else 0.0,
            right_speed      = right_speed if moving else 0.0,
            speed_multiplier = multiplier,
            reason           = reason,
        )

    def _stop(self, reason: str) -> ConvoyCommand:
        return ConvoyCommand(
            should_move      = False,
            left_speed       = 0.0,
            right_speed      = 0.0,
            speed_multiplier = 0.0,
            reason           = reason,
        )

    def _clamp(self, value: float) -> float:
        return max(0.0, min(float(value), self.max_speed))