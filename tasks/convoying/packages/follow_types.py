from dataclasses import dataclass, field
from typing import List, Optional, Tuple

BBox = Tuple[int, int, int, int]

FAR = "FAR"
GOOD = "GOOD"
CLOSE = "CLOSE"
TOO_CLOSE = "TOO_CLOSE"
LOST = "LOST"


@dataclass
class TargetInfo:
    found: bool
    bbox: Optional[BBox]
    center_x: Optional[float]
    center_y: Optional[float]
    bottom_y: Optional[int]
    area: int
    score: float
    class_id: Optional[int]
    distance_state: str
    reason: str
    # Dot-grid fields (optional — default-safe so old code still works)
    dot_count: int = 0
    dot_column: Optional[int] = None          # 1-7; 4 = centre
    dot_centers: List[Tuple[float, float]] = field(default_factory=list)
    tracking_state: str = "SEARCH"            # DOTS / GRACE / SEARCH


@dataclass
class ConvoyCommand:
    should_move: bool
    left_speed: float
    right_speed: float
    speed_multiplier: float
    reason: str