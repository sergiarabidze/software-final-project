import sys
import os
import signal
import threading
import time
import queue
import socket

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '..', '..')
sys.path.insert(0, project_root)

import cv2
from flask import Flask, Response, render_template_string, jsonify, request

from tasks.object_detection.packages.agent import ObjectDetectionAgent
from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
from tasks.convoying.packages.target_tracker_activity import TargetTracker
from tasks.convoying.packages.convoy_controller_activity import ConvoyController
from tasks.convoying.packages.red_line_gate import RedLineGate

from servers.convoying.visualization import create_convoying_visualization
from servers.templates.convoying import CONVOYING_TEMPLATE as HTML_TEMPLATE

from duckiebot.camera_driver import CameraDriver
from duckiebot.wheel_driver import DaguWheelsDriver
from duckiebot.wheel_driver.wheels_driver_abs import WheelPWMConfiguration
from launcher.ports import find_available_port
from servers.common import make_frame_generator, shutdown_cleanup, suppress_http_logs


app = Flask(__name__)

camera = None
wheels = None
object_agent = None
lane_agent = None
tracker = None
convoy_ctrl = None
red_line_gate = None

running = False
manual_mode = False
keys_pressed = {
    'up': False,
    'down': False,
    'left': False,
    'right': False,
}
_keys_lock = threading.Lock()
_keys_last_update = time.time()
stop_event = threading.Event()

# Async detection — keeps the video stream smooth while YOLO runs.
_frame_queue = queue.Queue(maxsize=1)
_detection_lock = threading.Lock()
_last_detections = []  # scaled to full camera resolution

_last_target = None
_last_valid_target = None
_last_command = None
_last_lane_left = 0.0
_last_lane_right = 0.0
_last_red_line_state = None

# Prints only when lane mode changes.
# False = normal lane + leader following.
# True  = lane disabled, leader-only following.
_last_lane_disabled = False


# ---------------------------------------------------------------------------
# Background detection thread
# ---------------------------------------------------------------------------

def _detection_loop():
    """Pull frames from the queue, run YOLO, store scaled detections."""
    global _last_detections

    while not stop_event.is_set():
        if object_agent is None or not object_agent.model_loaded:
            time.sleep(0.1)
            continue

        try:
            frame_rgb, orig_h, orig_w = _frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        result = object_agent.detect(frame_rgb)
        if result is None:
            continue

        # Scale boxes from model input resolution back to full camera size.
        sx = orig_w / float(object_agent.img_size)
        sy = orig_h / float(object_agent.img_size)

        scaled = [
            ((int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)), score, class_id)
            for (x1, y1, x2, y2), score, class_id in result
        ]

        with _detection_lock:
            _last_detections = scaled


# ---------------------------------------------------------------------------
# Manual drive extension
# ---------------------------------------------------------------------------

def _clear_manual_keys():
    """Release all manual-drive keys. Used on stop/reset/mode switch."""
    with _keys_lock:
        for key in keys_pressed:
            keys_pressed[key] = False


def _manual_drive_speeds(forward_speed, reverse_speed, turn_speed):
    """Convert current keyboard state to left/right wheel speeds."""
    global _keys_last_update

    # Safety: if the browser stops sending key updates, stop the robot.
    if time.time() - _keys_last_update > 0.5:
        _clear_manual_keys()

    with _keys_lock:
        keys = keys_pressed.copy()

    left = 0.0
    right = 0.0

    if keys['up']:
        left, right = forward_speed, forward_speed

    if keys['down']:
        left, right = -reverse_speed, -reverse_speed

    if keys['up'] and keys['left']:
        left, right = forward_speed * 0.45, forward_speed
    elif keys['up'] and keys['right']:
        left, right = forward_speed, forward_speed * 0.45
    elif keys['left']:
        left, right = -turn_speed, turn_speed
    elif keys['right']:
        left, right = turn_speed, -turn_speed

    return left, right


def _manual_control_loop(forward_speed, reverse_speed, turn_speed):
    """Own wheel commands while manual mode is active."""
    while not stop_event.is_set():
        if not manual_mode or not running or wheels is None:
            time.sleep(0.05)
            continue

        try:
            game_over = wheels.is_game_over() if hasattr(wheels, 'is_game_over') else False
            if game_over:
                wheels.set_wheels_speed(0.0, 0.0)
            else:
                left, right = _manual_drive_speeds(forward_speed, reverse_speed, turn_speed)
                wheels.set_wheels_speed(left, right)
        except Exception as exc:
            print(f"[ManualDrive] wheel command failed: {exc}")
            time.sleep(0.1)
            continue

        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Frame pipeline (called for every MJPEG frame)
# ---------------------------------------------------------------------------

def visualize(frame_bgr):
    """
    frame_bgr: BGR frame from the real camera (CameraDriver.read() returns BGR).

    Pipeline:
        camera frame
        -> red line gate
        -> object detection (async thread)
        -> target tracking
        -> lane servoing if allowed
        -> convoy controller
        -> wheel speeds
        -> browser visualization
    """
    global _last_target
    global _last_valid_target
    global _last_command
    global _last_lane_left
    global _last_lane_right
    global _last_red_line_state
    global _last_lane_disabled

    if frame_bgr is None:
        return frame_bgr

    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    image_height, image_width = frame_bgr.shape[:2]

    # Red line detection.
    # If close red line is detected, lane following is disabled for 5 seconds.
    if red_line_gate is not None:
        red_line_state = red_line_gate.update(frame_rgb)
    else:
        red_line_state = None

    _last_red_line_state = red_line_state
    lane_disabled = bool(red_line_state and red_line_state.lane_disabled)

    # Print only when mode changes.
    if lane_disabled != _last_lane_disabled:
        if lane_disabled:
            remaining = getattr(red_line_state, "disabled_remaining", 0.0)
            red_ratio = getattr(red_line_state, "red_ratio", 0.0)
            red_row_ratio = getattr(red_line_state, "red_row_ratio", 0.0)

            print(
                "[Convoying] LANE DETECTION OFF -> LEADER-ONLY FOLLOWING ON "
                f"| remaining={remaining:.2f}s "
                f"| red_ratio={red_ratio:.4f} "
                f"| red_row_ratio={red_row_ratio:.4f}"
            )
        else:
            print("[Convoying] LANE DETECTION ON -> normal lane + leader following resumed")

        _last_lane_disabled = lane_disabled

    # Feed a downscaled copy to the detection thread (non-blocking).
    if object_agent is not None and object_agent.model_loaded:
        small = cv2.resize(frame_rgb, (object_agent.img_size, object_agent.img_size))
        try:
            _frame_queue.put_nowait((small, image_height, image_width))
        except queue.Full:
            pass

    # Grab latest detections. They are already scaled to full camera resolution.
    with _detection_lock:
        detections = list(_last_detections)

    # Target tracking — camera-based dot-grid detection, no object model needed.
    if tracker is not None:
        target = tracker.update(
            frame_rgb=frame_rgb,
            image_height=image_height,
            image_width=image_width,
        )
    else:
        target = _last_target

    if target is not None and target.found:
        _last_valid_target = target

    # Lane servoing.
    # Important: when lane_disabled is True, do NOT use lane commands.
    # The controller will steer only toward the leader truck.
    if lane_agent is not None and not lane_disabled:
        try:
            lane_left, lane_right = lane_agent.compute_commands(frame_rgb)
        except Exception as e:
            print(f"[Convoying] Lane servoing error: {e}")
            lane_left, lane_right = 0.0, 0.0
    else:
        lane_left, lane_right = 0.0, 0.0

    # Convoy controller.
    # Normal mode: lane_left/lane_right steer the bot.
    # Red-line mode: lane_disabled=True, controller ignores lane and follows leader only.
    if convoy_ctrl is not None and target is not None:
        command = convoy_ctrl.decide(
            target=target,
            lane_left=lane_left,
            lane_right=lane_right,
            image_width=image_width,
            lane_disabled=lane_disabled,
        )
    else:
        command = _last_command

    # Apply wheel commands only when running.
    # Manual mode owns the wheels in _manual_control_loop(), so the
    # autonomous convoy controller must not fight it here.
    if wheels is not None:
        if manual_mode:
            pass
        elif running and command is not None:
            wheels.set_wheels_speed(command.left_speed, command.right_speed)
        else:
            wheels.set_wheels_speed(0.0, 0.0)

    _last_target = target
    _last_command = command
    _last_lane_left = lane_left
    _last_lane_right = lane_right

    return create_convoying_visualization(
        image_bgr=frame_bgr,
        lane_debug_info=lane_agent.last_debug_info if lane_agent else {},
        target=target,
        command=command,
        lane_left=lane_left,
        lane_right=lane_right,
        detections=detections,
    )


# Real camera returns BGR frames — rgb=False.
generate_frames = make_frame_generator(lambda: camera, visualize, quality=50, rgb=False)


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, hostname=socket.gethostname())


@app.route('/video')
def video():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )


@app.route('/start', methods=['POST'])
def start():
    global running

    running = True
    print("[Convoying] Started")
    return jsonify({'status': 'running'})


@app.route('/stop', methods=['POST'])
def stop():
    global running

    running = False

    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)

    _clear_manual_keys()

    print("[Convoying] Stopped")
    return jsonify({'status': 'stopped'})


@app.route('/reset', methods=['POST'])
def reset():
    global running
    global _last_detections
    global _last_target
    global _last_valid_target
    global _last_command
    global _last_red_line_state
    global _last_lane_disabled

    running = False

    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)

    _clear_manual_keys()

    if tracker is not None:
        tracker.reset()

    if red_line_gate is not None:
        red_line_gate.reset()

    with _detection_lock:
        _last_detections = []

    _last_target = None
    _last_valid_target = None
    _last_command = None
    _last_red_line_state = None
    _last_lane_disabled = False

    print("[Convoying] Reset")
    return jsonify({'status': 'reset'})



@app.route('/set_mode', methods=['POST'])
def set_mode():
    """Switch between autonomous convoying and manual keyboard drive."""
    global manual_mode

    data = request.json or {}
    mode = data.get('mode', 'auto')
    manual_mode = mode == 'manual'

    _clear_manual_keys()

    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)

    current_mode = 'manual' if manual_mode else 'auto'
    print(f"[Convoying] Mode: {current_mode}")

    return jsonify({
        'status': current_mode,
        'mode': current_mode,
        'manual_mode': manual_mode,
        'running': running,
    })


@app.route('/manual/enable', methods=['POST'])
def manual_enable():
    """Alias for set_mode(manual) — keeps compatibility with older templates."""
    global manual_mode
    manual_mode = True
    _clear_manual_keys()
    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)
    print("[Convoying] Mode: manual (via /manual/enable)")
    return jsonify({'status': 'manual', 'manual_mode': True})


@app.route('/manual/disable', methods=['POST'])
def manual_disable():
    """Alias for set_mode(auto) — keeps compatibility with older templates."""
    global manual_mode
    manual_mode = False
    _clear_manual_keys()
    if wheels is not None:
        wheels.set_wheels_speed(0.0, 0.0)
    print("[Convoying] Mode: auto (via /manual/disable)")
    return jsonify({'status': 'auto', 'manual_mode': False})


@app.route('/keys', methods=['POST'])
def update_keys():
    """Receive keyboard state from the dashboard while manual mode is active."""
    global _keys_last_update

    data = request.json or {}

    with _keys_lock:
        for key in keys_pressed:
            keys_pressed[key] = bool(data.get(key, False))

    _keys_last_update = time.time()

    return jsonify({'status': 'ok'})


@app.route('/running')
def get_running():
    return jsonify({'running': running})


@app.route('/status')
def status():
    return jsonify({
        'running': running,
        'manual_mode': manual_mode,
        'mode': 'manual' if manual_mode else 'auto',
        'model_loaded': bool(getattr(object_agent, 'model_loaded', False)) if object_agent else False,
        'model_load_error': getattr(object_agent, 'load_error', None) if object_agent else None,
        'trt_building': getattr(object_agent, 'trt_building', False) if object_agent else False,
        'lane_frame_count': getattr(lane_agent, 'frame_count', 0) if lane_agent else 0,
        'target': _target_to_dict(_last_target),
        'command': _command_to_dict(_last_command),
        'lane_left': _last_lane_left,
        'lane_right': _last_lane_right,
        'lane_disabled': _last_lane_disabled,
        'red_line': _red_line_to_dict(_last_red_line_state),
    })


@app.route('/update_config', methods=['POST'])
def update_config():
    """Tune convoy controller multipliers live from the browser."""
    data = request.json or {}

    if convoy_ctrl is not None:
        if 'close_multiplier' in data:
            convoy_ctrl.close_multiplier = float(data['close_multiplier'])
        if 'good_multiplier' in data:
            convoy_ctrl.good_multiplier = float(data['good_multiplier'])
        if 'far_multiplier' in data:
            convoy_ctrl.far_multiplier = float(data['far_multiplier'])
        if 'max_speed' in data:
            convoy_ctrl.max_speed = float(data['max_speed'])

        # Optional leader-only tuning from browser/API.
        if 'leader_steering_gain' in data:
            convoy_ctrl.leader_steering_gain = float(data['leader_steering_gain'])
        if 'leader_steering_sign' in data:
            convoy_ctrl.leader_steering_sign = float(data['leader_steering_sign'])

    return jsonify({
        'close_multiplier': convoy_ctrl.close_multiplier if convoy_ctrl else None,
        'good_multiplier': convoy_ctrl.good_multiplier if convoy_ctrl else None,
        'far_multiplier': convoy_ctrl.far_multiplier if convoy_ctrl else None,
        'max_speed': convoy_ctrl.max_speed if convoy_ctrl else None,
        'leader_steering_gain': convoy_ctrl.leader_steering_gain if convoy_ctrl else None,
        'leader_steering_sign': convoy_ctrl.leader_steering_sign if convoy_ctrl else None,
    })


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _target_to_dict(target):
    if target is None:
        return None

    return {
        'found': target.found,
        'bbox': target.bbox,
        'center_x': target.center_x,
        'center_y': target.center_y,
        'bottom_y': target.bottom_y,
        'area': target.area,
        'score': target.score,
        'class_id': target.class_id,
        'distance_state': target.distance_state,
        'reason': target.reason,
    }


def _command_to_dict(command):
    if command is None:
        return None

    return {
        'should_move': command.should_move,
        'left_speed': command.left_speed,
        'right_speed': command.right_speed,
        'speed_multiplier': command.speed_multiplier,
        'reason': command.reason,
    }


def _red_line_to_dict(red_line_state):
    if red_line_state is None:
        return None

    return {
        'red_line_close': red_line_state.red_line_close,
        'lane_disabled': red_line_state.lane_disabled,
        'disabled_remaining': red_line_state.disabled_remaining,
        'red_ratio': red_line_state.red_ratio,
        'red_row_ratio': red_line_state.red_row_ratio,
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    global camera
    global wheels
    global object_agent
    global lane_agent
    global tracker
    global convoy_ctrl
    global red_line_gate

    import argparse
    ap = argparse.ArgumentParser(description="Real Convoying Server")
    ap.add_argument('--port', type=int, default=5000)
    args = ap.parse_args()

    suppress_http_logs()

    print("=" * 60)
    print("CONVOYING — REAL ROBOT SERVER")
    print("=" * 60)

    def _init_wheels():
        global wheels
        wheels = DaguWheelsDriver(WheelPWMConfiguration(), WheelPWMConfiguration())
        print("[Init] Wheels ready")

    def _init_camera():
        global camera
        cam = CameraDriver()
        cam.start()
        camera = cam
        print("[Init] Camera ready")

    def _init_agents():
        global object_agent
        global lane_agent
        global tracker
        global convoy_ctrl
        global red_line_gate

        lane_agent = LaneServoingAgent()
        print(f"[Init] Lane agent ready (base_speed={lane_agent.base_speed})")

        object_agent = ObjectDetectionAgent()
        if object_agent.model_loaded:
            print(f"[Init] Detection model ready ({object_agent.img_size}px)")
        elif getattr(object_agent, 'trt_building', False):
            print("[Init] TensorRT engine building in background — will be ready shortly")
        else:
            print(f"[Init] Detection model: {object_agent.load_error}")

        tracker = TargetTracker()
        convoy_ctrl = ConvoyController()
        red_line_gate = RedLineGate(disable_seconds=5.0)

        print("[Init] Convoy tracker and controller ready")
        print("[Init] Red-line gate ready: lane disabled for 5 seconds after close red line")

    threading.Thread(target=_init_wheels, daemon=True).start()
    threading.Thread(target=_init_camera, daemon=True).start()
    threading.Thread(target=_init_agents, daemon=True).start()
    threading.Thread(target=_detection_loop, daemon=True).start()
    threading.Thread(
        target=_manual_control_loop,
        args=(0.25, 0.20, 0.18),
        daemon=True,
    ).start()

    def _shutdown(signum, frame):
        shutdown_cleanup(wheels, camera, stop_event)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    web_port = find_available_port(args.port)
    print(f"\nWeb Interface: http://{socket.gethostname()}.local:{web_port}")
    print("=" * 60 + "\n")

    try:
        app.run(host='0.0.0.0', port=web_port, debug=False, threaded=True)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        shutdown_cleanup(wheels, camera, stop_event)


if __name__ == '__main__':
    sys.exit(main())