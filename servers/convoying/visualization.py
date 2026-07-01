import cv2
import numpy as np


def create_convoying_visualization(
    image_bgr: np.ndarray,
    lane_debug_info: dict,
    target,
    command,
    lane_left: float,
    lane_right: float,
    detections,
) -> np.ndarray:
    display_w = 360
    h, w = image_bgr.shape[:2]
    display_h = int(h * display_w / w)

    camera_panel = cv2.resize(image_bgr, (display_w, display_h))

    _draw_detections(camera_panel, detections, w, h, display_w, display_h)
    _draw_target(camera_panel, target, w, h, display_w, display_h)

    lane_panel = _make_lane_panel(lane_debug_info, display_w, display_h)
    info_panel = _make_info_panel(
        width=display_w * 2,
        target=target,
        command=command,
        lane_left=lane_left,
        lane_right=lane_right,
    )

    top = np.hstack([camera_panel, lane_panel])
    return np.vstack([top, info_panel])


def _draw_detections(panel, detections, orig_w, orig_h, display_w, display_h):
    if not detections:
        return

    sx = display_w / float(orig_w)
    sy = display_h / float(orig_h)

    for bbox, score, class_id in detections:
        x1, y1, x2, y2 = bbox

        dx1 = int(x1 * sx)
        dy1 = int(y1 * sy)
        dx2 = int(x2 * sx)
        dy2 = int(y2 * sy)

        color = (255, 100, 100)
        if class_id == 1:
            color = (0, 255, 0)

        cv2.rectangle(panel, (dx1, dy1), (dx2, dy2), color, 1)
        cv2.putText(
            panel,
            f"id:{class_id} {score:.2f}",
            (dx1, max(15, dy1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            color,
            1,
        )


def _draw_target(panel, target, orig_w, orig_h, display_w, display_h):
    if target is None or not target.found:
        cv2.putText(
            panel,
            "TARGET LOST",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        return

    sx = display_w / float(orig_w)
    sy = display_h / float(orig_h)

    dot_centers = getattr(target, 'dot_centers', [])
    dot_column  = getattr(target, 'dot_column', None)
    tracking_state = getattr(target, 'tracking_state', '')

    # ---- draw detected dots ----
    for (dx, dy) in dot_centers:
        px = int(dx * sx)
        py = int(dy * sy)
        cv2.circle(panel, (px, py), 5, (0, 200, 255), -1)     # filled orange-ish
        cv2.circle(panel, (px, py), 6, (255, 255, 255), 1)    # white ring

    # ---- draw 7-column grid over the dot cluster ----
    if dot_centers:
        xs = [dx * sx for dx, _ in dot_centers]
        ys = [dy * sy for _, dy in dot_centers]
        min_x = max(0, int(min(xs)) - 10)
        max_x = min(display_w, int(max(xs)) + 10)
        min_y = max(0, int(min(ys)) - 10)
        max_y = min(display_h, int(max(ys)) + 10)

        span = max(max_x - min_x, 1)
        col_w = span / 7.0

        for i in range(8):                            # 8 lines → 7 gaps
            lx = int(min_x + i * col_w)
            color = (180, 180, 180) if i not in (0, 7) else (100, 100, 100)
            cv2.line(panel, (lx, min_y), (lx, max_y), color, 1)

        for i in range(7):                            # labels 1-7
            col_num = i + 1
            lx = int(min_x + (i + 0.5) * col_w)
            is_active = (dot_column == col_num)
            bg_color  = (0, 220, 0) if is_active else (60, 60, 60)
            txt_color = (0, 0, 0)   if is_active else (180, 180, 180)
            cv2.rectangle(panel, (lx - 8, min_y), (lx + 8, min_y + 16), bg_color, -1)
            cv2.putText(panel, str(col_num), (lx - 4, min_y + 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1)

        # bounding box around cluster
        cv2.rectangle(panel, (min_x, min_y), (max_x, max_y), (0, 255, 255), 1)

    elif target.center_x is not None:
        # grace period — no dots but held position
        cx = int(target.center_x * sx)
        cy = int(target.center_y * sy) if target.center_y else display_h // 2
        cv2.drawMarker(panel, (cx, cy), (0, 180, 255),
                       cv2.MARKER_CROSS, 20, 2)

    # ---- status label ----
    state_label = f"COL {dot_column}  {tracking_state}" if dot_column else tracking_state
    col = (0, 255, 0) if tracking_state == "DOTS" else (0, 200, 255) if tracking_state == "GRACE" else (0, 0, 255)
    cv2.putText(panel, state_label, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)


def _make_lane_panel(lane_debug_info, display_w, display_h):
    if not lane_debug_info or 'lane_mask' not in lane_debug_info:
        panel = np.zeros((display_h, display_w, 3), dtype=np.uint8)
        cv2.putText(
            panel,
            "No lane debug",
            (20, display_h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (100, 100, 100),
            2,
        )
        return panel

    lane_mask = lane_debug_info.get('lane_mask')
    lane_panel = cv2.applyColorMap(lane_mask, cv2.COLORMAP_HOT)
    lane_panel = cv2.resize(lane_panel, (display_w, display_h))

    cv2.putText(
        lane_panel,
        "Lane Mask",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )

    return lane_panel


def _make_info_panel(width, target, command, lane_left, lane_right):
    height = 150
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    if target is None:
        target_text = "Target: none"
        distance_text = "Distance: none"
        reason_text = "Reason: no target object"
    else:
        dot_column = getattr(target, 'dot_column', None)
        dot_count  = getattr(target, 'dot_count', 0)
        tracking_state = getattr(target, 'tracking_state', '')
        target_text = f"Target: {tracking_state}  dots={dot_count}  col={dot_column if dot_column else '-'}"
        distance_text = f"Distance: {target.distance_state}"
        reason_text = f"Reason: {target.reason}"

    if command is None:
        command_text = "Command: none"
        speed_text = f"Lane L/R: {lane_left:.3f} / {lane_right:.3f}"
        multiplier_text = "Multiplier: none"
    else:
        command_text = f"Move: {command.should_move} | {command.reason}"
        speed_text = (
            f"Lane L/R: {lane_left:.3f} / {lane_right:.3f}    "
            f"Final L/R: {command.left_speed:.3f} / {command.right_speed:.3f}"
        )
        multiplier_text = f"Multiplier: {command.speed_multiplier:.2f}"

    lines = [
        target_text,
        distance_text,
        reason_text,
        command_text,
        speed_text,
        multiplier_text,
    ]

    y = 22
    for line in lines:
        cv2.putText(panel, line, (10, y), font, 0.45, (220, 220, 220), 1)
        y += 22

    return panel