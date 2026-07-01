"""
Dot-grid target tracker for convoy following.

The leader's backplate carries a 3×7 grid of orange dots.
This tracker:
  1. Detects dots every frame (orange LEDs first, dark blobs as fallback).
  2. Assigns each dot to one of 7 virtual columns based on its x-position
     within the dot cluster bounding box.
  3. Computes the weighted column centre (1–7; 4 = straight ahead).
  4. Persists the last column reading across short loss periods so the
     follower keeps turning in the correct direction after the dots
     temporarily leave the frame.
  5. Returns a TargetInfo whose center_x drives the convoy controller's
     existing steering logic unchanged.
"""

from typing import List, Optional, Tuple
import math

import cv2
import numpy as np

from tasks.convoying.packages.follow_types import (
    TargetInfo,
    FAR,
    GOOD,
    CLOSE,
    TOO_CLOSE,
    LOST,
)

NUM_COLUMNS = 7
_CENTER_COLUMN = (NUM_COLUMNS + 1) / 2.0   # 4.0
Detection = Tuple[Tuple[int, int, int, int], float, int]


class TargetTracker:
    """Tracks the leader truck via the 3×7 dot grid on its backplate."""

    def __init__(
        self,
        min_dot_radius: int = 2,
        max_dot_radius: int = 22,
        min_dots_required: int = 4,
        grace_frames: int = 20,
        target_class_ids: Tuple[int, ...] = (1,),
        rejected_class_ids: Tuple[int, ...] = (2,),
        min_score: float = 0.20,
        min_area: int = 800,
        max_center_shift_ratio: float = 0.70,
        initial_center_window_ratio: float = 0.52,
        min_bottom_ratio: float = 0.32,
        # Distance thresholds: bottom_y / image_height
        too_close_ratio: float = 0.75,
        close_ratio: float = 0.62,
        good_ratio: float = 0.44,
    ):
        self.min_dot_radius = min_dot_radius
        self.max_dot_radius = max_dot_radius
        self.min_dots_required = min_dots_required
        self.grace_frames = grace_frames
        self.target_class_ids = target_class_ids
        self.rejected_class_ids = rejected_class_ids
        self.min_score = min_score
        self.min_area = min_area
        self.max_center_shift_ratio = max_center_shift_ratio
        self.initial_center_window_ratio = initial_center_window_ratio
        self.min_bottom_ratio = min_bottom_ratio
        self.too_close_ratio = too_close_ratio
        self.close_ratio = close_ratio
        self.good_ratio = good_ratio

        # Persistent state
        self._last_dot_column: Optional[int] = None   # 1-7
        self._last_center_x: Optional[float] = None   # pixels
        self._last_center_y: Optional[float] = None
        self._lost_frames: int = 0
        self._had_target: bool = False

    def reset(self) -> None:
        self._last_dot_column = None
        self._last_center_x = None
        self._last_center_y = None
        self._lost_frames = 0
        self._had_target = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        frame_rgb: Optional[np.ndarray] = None,
        image_height: int = None,
        image_width: Optional[int] = None,
        detections: Optional[list] = None,  # backwards compat for old API
        **_kwargs,
    ) -> TargetInfo:
        # Backwards compatibility: if called with old API (detections=...), ignore detections
        # and just use frame_rgb if provided, or return lost if not
        if frame_rgb is None:
            if detections is not None:
                if image_height is None:
                    raise ValueError("image_height is required")
                if image_width is None:
                    image_width = image_height
                return self._update_from_detections(
                    detections=detections,
                    image_height=image_height,
                    image_width=image_width,
                )
            raise ValueError("frame_rgb is required")

        if image_height is None:
            raise ValueError("image_height is required")

        if image_width is None:
            image_width = image_height

        dots = _detect_dots(frame_rgb, self.min_dot_radius, self.max_dot_radius)
        dots = _apply_roi_filter(
            dots=dots,
            image_width=image_width,
            image_height=image_height,
            last_center_x=self._last_center_x,
            last_center_y=self._last_center_y,
        )
        dots = _select_best_cluster(
            dots=dots,
            image_width=image_width,
            image_height=image_height,
            last_center_x=self._last_center_x,
            last_center_y=self._last_center_y,
        )

        if len(dots) >= self.min_dots_required:
            return self._build_from_dots(dots, image_height, image_width)

        # ---- dots lost ----
        self._lost_frames += 1

        if self._had_target and self._lost_frames <= self.grace_frames and self._last_center_x is not None:
            # Hold last position so the controller keeps steering the right way.
            bottom_y = int(self._last_center_y) if self._last_center_y is not None else image_height // 2
            distance_state = self._distance_state(bottom_y, image_height)
            print(
                f"[TargetTracker] GRACE frame={self._lost_frames}/{self.grace_frames} "
                f"col={self._last_dot_column}"
            )
            return TargetInfo(
                found=True,
                bbox=None,
                center_x=self._last_center_x,
                center_y=self._last_center_y,
                bottom_y=bottom_y,
                area=0,
                score=0.0,
                class_id=None,
                distance_state=distance_state,
                reason="dot_grace_period",
                dot_count=0,
                dot_column=self._last_dot_column,
                dot_centers=[],
                tracking_state="GRACE",
            )

        print("[TargetTracker] SEARCH – no dots found")
        return TargetInfo(
            found=False,
            bbox=None,
            center_x=None,
            center_y=None,
            bottom_y=None,
            area=0,
            score=0.0,
            class_id=None,
            distance_state=LOST,
            reason="no_dots_detected",
            dot_count=0,
            dot_column=self._last_dot_column,
            dot_centers=[],
            tracking_state="SEARCH",
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_from_dots(
        self,
        dots: List[Tuple[float, float, float]],
        image_height: int,
        image_width: int,
    ) -> TargetInfo:
        self._lost_frames = 0
        self._had_target = True

        xs = [cx for cx, _cy, _r in dots]
        ys = [cy for _cx, cy, _r in dots]
        radii = [r for _cx, _cy, r in dots]

        center_x = float(sum(xs) / len(xs))
        center_y = float(sum(ys) / len(ys))
        bottom_y = int(max(ys) + max(radii))

        # --- assign column 1-7 based on x within cluster bounding box ---
        min_x, max_x = min(xs), max(xs)
        span = max(max_x - min_x, 1.0)

        def _col(x: float) -> int:
            ratio = (x - min_x) / span          # 0..1
            return max(1, min(NUM_COLUMNS, int(ratio * NUM_COLUMNS) + 1))

        dot_columns = [_col(x) for x in xs]
        # Weighted by area (radius²)
        weighted_sum = sum(c * r * r for c, r in zip(dot_columns, radii))
        weight_total = sum(r * r for r in radii)
        col_f = weighted_sum / weight_total if weight_total > 0 else _CENTER_COLUMN
        dot_column = max(1, min(NUM_COLUMNS, round(col_f)))

        self._last_center_x = center_x
        self._last_center_y = center_y
        self._last_dot_column = dot_column

        distance_state = self._distance_state(bottom_y, image_height)
        dot_centers = [(cx, cy) for cx, cy, _ in dots]

        print(
            f"[TargetTracker] DOTS found={len(dots)} col={dot_column} "
            f"cx={center_x:.0f}/{image_width} dist={distance_state}"
        )

        return TargetInfo(
            found=True,
            bbox=None,
            center_x=center_x,
            center_y=center_y,
            bottom_y=bottom_y,
            area=int(sum((2 * r) ** 2 for r in radii)),
            score=1.0,
            class_id=None,
            distance_state=distance_state,
            reason="dot_grid_detected",
            dot_count=len(dots),
            dot_column=dot_column,
            dot_centers=dot_centers,
            tracking_state="DOTS",
        )

    def _distance_state(self, bottom_y: int, image_height: int) -> str:
        if image_height <= 0:
            return LOST
        r = bottom_y / float(image_height)
        if r >= self.too_close_ratio:
            return TOO_CLOSE
        if r >= self.close_ratio:
            return CLOSE
        if r >= self.good_ratio:
            return GOOD
        return FAR

    def _update_from_detections(
        self,
        detections: Optional[List[Detection]],
        image_height: int,
        image_width: int,
    ) -> TargetInfo:
        if detections is None or len(detections) == 0:
            return self._lost_detection("target_lost_no_detections")

        valid = [
            d for d in detections
            if self._is_valid_target(d)
            and self._is_front_candidate(d, image_width, image_height)
        ]

        # If no strict front-candidate exists, allow a relaxed pass but keep
        # center/bottom bias in scorer so side objects are still disfavored.
        if not valid:
            valid = [d for d in detections if self._is_valid_target(d)]

        if not valid:
            return self._lost_detection(self._build_no_target_reason(detections))

        selected = self._match_previous_detection(valid, image_width, image_height)
        if selected is None:
            return self._lost_detection("previous_target_not_matched")

        return self._build_target_info_from_detection(selected, image_height)

    def _is_valid_target(self, detection: Detection) -> bool:
        bbox, score, class_id = detection
        if class_id in self.rejected_class_ids:
            return False
        if class_id not in self.target_class_ids:
            return False
        if score < self.min_score:
            return False
        if self._bbox_area(bbox) < self.min_area:
            return False
        return True

    def _match_previous_detection(
        self,
        detections: List[Detection],
        image_width: int,
        image_height: int,
    ) -> Optional[Detection]:
        if self._last_center_x is None or self._last_center_y is None:
            return self._choose_initial_detection(detections, image_width, image_height)

        best_detection = None
        best_distance = float("inf")

        for detection in detections:
            bbox, _, _ = detection
            center_x, center_y = self._bbox_center(bbox)
            distance = math.hypot(center_x - self._last_center_x, center_y - self._last_center_y)
            if distance < best_distance:
                best_distance = distance
                best_detection = detection

        image_diagonal = math.hypot(image_width, image_height)
        max_allowed_shift = image_diagonal * self.max_center_shift_ratio
        if best_distance > max_allowed_shift:
            return None
        return best_detection

    def _choose_initial_detection(
        self,
        detections: List[Detection],
        image_width: int,
        image_height: int,
    ) -> Detection:
        half = image_width / 2.0
        frame_area = float(max(1, image_width * image_height))

        def _score(d: Detection) -> float:
            bbox, conf, _ = d
            cx, _ = self._bbox_center(bbox)
            area = self._bbox_area(bbox)
            bottom_y = bbox[3]

            center_penalty = abs(cx - half) / max(half, 1.0)
            bottom_ratio = bottom_y / float(max(1, image_height))
            area_ratio = area / frame_area

            # Prefer objects straight ahead and closer to the robot.
            return (
                2.4 * bottom_ratio
                + 1.4 * area_ratio
                + 0.3 * float(conf)
                - 1.9 * center_penalty
            )

        return max(detections, key=_score)

    def _is_front_candidate(
        self,
        detection: Detection,
        image_width: int,
        image_height: int,
    ) -> bool:
        bbox, _, _ = detection
        cx, _ = self._bbox_center(bbox)
        bottom_ratio = bbox[3] / float(max(1, image_height))

        half = image_width / 2.0
        center_offset = abs(cx - half) / max(half, 1.0)

        if self._last_center_x is None or self._last_center_y is None:
            # Initial lock: only accept reasonably centered, forward objects.
            if center_offset > self.initial_center_window_ratio:
                return False
            if bottom_ratio < self.min_bottom_ratio:
                return False
            return True

        # During tracking we allow wider motion but still reject edge objects.
        return center_offset <= 0.92

    def _build_target_info_from_detection(
        self,
        detection: Detection,
        image_height: int,
    ) -> TargetInfo:
        bbox, score, class_id = detection
        _, _, _, bottom_y = bbox
        center_x, center_y = self._bbox_center(bbox)
        area = self._bbox_area(bbox)

        self._last_center_x = center_x
        self._last_center_y = center_y
        self._had_target = True
        self._lost_frames = 0

        return TargetInfo(
            found=True,
            bbox=bbox,
            center_x=center_x,
            center_y=center_y,
            bottom_y=bottom_y,
            area=area,
            score=float(score),
            class_id=int(class_id),
            distance_state=self._distance_state(bottom_y, image_height),
            reason="target_tracked_from_detections",
            dot_count=0,
            dot_column=self._last_dot_column,
            dot_centers=[],
            tracking_state="DETECTIONS",
        )

    def _lost_detection(self, reason: str) -> TargetInfo:
        return TargetInfo(
            found=False,
            bbox=None,
            center_x=None,
            center_y=None,
            bottom_y=None,
            area=0,
            score=0.0,
            class_id=None,
            distance_state=LOST,
            reason=reason,
            dot_count=0,
            dot_column=self._last_dot_column,
            dot_centers=[],
            tracking_state="SEARCH",
        )

    def _build_no_target_reason(self, detections: List[Detection]) -> str:
        class_counts = {}
        for _, _, class_id in detections:
            class_counts[class_id] = class_counts.get(class_id, 0) + 1
        return f"no_valid_target_detection_classes={class_counts}"

    @staticmethod
    def _bbox_center(bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
        x1, y1, x2, y2 = bbox
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0

    @staticmethod
    def _bbox_area(bbox: Tuple[int, int, int, int]) -> int:
        x1, y1, x2, y2 = bbox
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        return width * height


# ---------------------------------------------------------------------------
# Dot detection – orange LEDs first, dark blobs as fallback
# ---------------------------------------------------------------------------

def _detect_dots(
    image_rgb: np.ndarray,
    min_radius: int,
    max_radius: int,
) -> List[Tuple[float, float, float]]:
    # 1. Orange/yellow LEDs (real Duckiebot backplate)
    dots = _detect_orange_dots(image_rgb, min_radius, max_radius)
    if dots:
        print(f"[Dots] Method=ORANGE found {len(dots)} dots")
        return dots

    gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 1.5)

    # 2. Hough circles
    dots = _detect_hough(blur, min_radius, max_radius)
    if dots:
        print(f"[Dots] Method=HOUGH found {len(dots)} dots")
        return dots

    # 3. Dark blobs via threshold
    dots = _detect_dark_blobs(blur, min_radius, max_radius)
    if dots:
        print(f"[Dots] Method=DARK_BLOBS found {len(dots)} dots")
        return dots

    # 4. Black HSV blobs
    dots = _detect_black_hsv(image_rgb, min_radius, max_radius)
    if dots:
        print(f"[Dots] Method=BLACK_HSV found {len(dots)} dots")
    else:
        print(f"[Dots] No dots found by any method (range={min_radius}-{max_radius})")
    return dots


def _detect_orange_dots(
    image_rgb: np.ndarray,
    min_radius: int,
    max_radius: int,
) -> List[Tuple[float, float, float]]:
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    # Wider orange range: lower saturation threshold, wider hue range
    lower1 = np.array([0,  100, 80])    # was [0, 120, 100]
    upper1 = np.array([25, 255, 255])   # was [22, 255, 255]
    lower2 = np.array([155, 100, 80])   # was [158, 120, 100]
    upper2 = np.array([180, 255, 255])
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower1, upper1),
        cv2.inRange(hsv, lower2, upper2),
    )
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    return _contour_dots(mask, min_radius, max_radius, circularity_thresh=0.15)


def _detect_hough(
    gray_blur: np.ndarray,
    min_radius: int,
    max_radius: int,
) -> List[Tuple[float, float, float]]:
    circles = cv2.HoughCircles(
        gray_blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(10, min_radius * 2),
        param1=80,
        param2=12,
        minRadius=min_radius,
        maxRadius=max_radius,
    )
    if circles is None:
        return []
    return [
        (float(cx), float(cy), float(r))
        for cx, cy, r in np.round(circles[0]).astype(int)
        if min_radius <= r <= max_radius
    ]


def _detect_dark_blobs(
    gray_blur: np.ndarray,
    min_radius: int,
    max_radius: int,
) -> List[Tuple[float, float, float]]:
    _, thresh = cv2.threshold(gray_blur, 130, 255, cv2.THRESH_BINARY_INV)  # was 120
    k = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  k, iterations=1)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, k, iterations=2)
    return _contour_dots(thresh, min_radius, max_radius, circularity_thresh=0.20)


def _detect_black_hsv(
    image_rgb: np.ndarray,
    min_radius: int,
    max_radius: int,
) -> List[Tuple[float, float, float]]:
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 120]))  # was 100
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
    return _contour_dots(mask, min_radius, max_radius, circularity_thresh=0.15)


def _contour_dots(
    binary: np.ndarray,
    min_radius: int,
    max_radius: int,
    circularity_thresh: float,
) -> List[Tuple[float, float, float]]:
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dots = []
    for c in contours:
        area = cv2.contourArea(c)
        if area <= 4:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter <= 0:
            continue
        if (4.0 * math.pi * area / (perimeter * perimeter)) < circularity_thresh:
            continue
        x, y, w, h = cv2.boundingRect(c)
        r = max(w, h) / 2.0
        if min_radius <= r <= max_radius:
            dots.append((float(x + w / 2.0), float(y + h / 2.0), r))
    return dots


def _apply_roi_filter(
    dots: List[Tuple[float, float, float]],
    image_width: int,
    image_height: int,
    last_center_x: Optional[float],
    last_center_y: Optional[float],
) -> List[Tuple[float, float, float]]:
    if not dots:
        return []

    if last_center_x is None or last_center_y is None:
        # Initial acquisition ROI: center-weighted and lower-biased.
        x_min = int(0.12 * image_width)
        x_max = int(0.88 * image_width)
        y_min = int(0.18 * image_height)
        y_max = int(0.98 * image_height)
    else:
        # Tracking ROI around the last known leader position.
        half_w = int(0.30 * image_width)
        half_h = int(0.26 * image_height)
        x_min = max(0, int(last_center_x) - half_w)
        x_max = min(image_width, int(last_center_x) + half_w)
        y_min = max(0, int(last_center_y) - half_h)
        y_max = min(image_height, int(last_center_y) + half_h)

    return [
        (cx, cy, r)
        for cx, cy, r in dots
        if x_min <= cx <= x_max and y_min <= cy <= y_max
    ]


def _select_best_cluster(
    dots: List[Tuple[float, float, float]],
    image_width: int,
    image_height: int,
    last_center_x: Optional[float],
    last_center_y: Optional[float],
) -> List[Tuple[float, float, float]]:
    if len(dots) <= 1:
        return dots

    # Group nearby dots into connected components in image space.
    max_link = max(18.0, 0.085 * math.hypot(image_width, image_height))
    n = len(dots)
    visited = [False] * n
    groups: List[List[Tuple[float, float, float]]] = []

    for i in range(n):
        if visited[i]:
            continue
        stack = [i]
        visited[i] = True
        idxs = []

        while stack:
            cur = stack.pop()
            idxs.append(cur)
            cx1, cy1, _ = dots[cur]
            for j in range(n):
                if visited[j]:
                    continue
                cx2, cy2, _ = dots[j]
                if math.hypot(cx2 - cx1, cy2 - cy1) <= max_link:
                    visited[j] = True
                    stack.append(j)

        groups.append([dots[k] for k in idxs])

    if len(groups) == 1:
        return groups[0]

    def _score(group: List[Tuple[float, float, float]]) -> float:
        xs = [x for x, _y, _r in group]
        ys = [y for _x, y, _r in group]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        aspect = width / max(height, 1.0)

        # Prefer many dots and compact horizontal clusters (truck backplate).
        count_score = float(len(group))
        aspect_penalty = abs(aspect - 2.3) * 0.8

        if last_center_x is not None and last_center_y is not None:
            dist_penalty = 0.015 * math.hypot(cx - last_center_x, cy - last_center_y)
        else:
            # Slight preference for clusters lower in the frame.
            dist_penalty = 0.006 * (image_height - cy)

        return count_score - aspect_penalty - dist_penalty

    best = max(groups, key=_score)
    return best