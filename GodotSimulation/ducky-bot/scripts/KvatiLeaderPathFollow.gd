extends PathFollow3D

# Path-based lead truck mover for KvatiTown convoying.
# The leader now stays parked until Python/dashboard sends a non-zero speed.
# It can also pause at hardcoded stop markers.

@export var speed: float = 0.0
@export var min_speed: float = 0.0
@export var max_speed: float = 0.25
@export var start_progress: float = 0.0
@export var loop_to_start: bool = false
@export var model_yaw_degrees: float = 180.0
@export var debug_print: bool = true

# Extra visual-only side offset. The real path is on the right-hand lane;
# negative pulls the visible truck away from the outer white border
# and closer to the center/yellow line in this scene.
@export var lane_side_offset: float = -0.13

# Lightly slow only the first tight left turn. Keep it simple/fast
# enough so the leader does not crawl through that first curve.
@export var first_turn_slowdown_enabled: bool = true
@export var first_turn_start_progress: float = 0.25
@export var first_turn_end_progress: float = 1.55
@export var first_turn_speed_multiplier: float = 0.78

# Slow the second tight left turn separately. The path itself is delayed
# in the scene so the leader reaches the stiff stop-sign turn a little later.
@export var second_turn_slowdown_enabled: bool = true
@export var second_turn_start_progress: float = 3.25
@export var second_turn_end_progress: float = 5.35
@export var second_turn_speed_multiplier: float = 0.62

# Stop shortly after the second/top-right turn.
# This locks the leader until Reset so repeated Start speed commands do not restart it.
@export var stop_after_second_turn_enabled: bool = true
@export var stop_after_second_turn_progress: float = 5.35

# After the second/top-right turn, shift only the visible truck toward the right-hand lane.
# This keeps the good start and first-turn offset unchanged.
@export var after_second_turn_right_offset_enabled: bool = false
@export var after_second_turn_offset_start_progress: float = 4.75
@export var after_second_turn_lane_side_offset: float = -0.13
@export var after_second_turn_offset_blend_distance: float = 0.55

# Hardcoded stop-sign behavior for the simulation map.
@export var stop_on_markers: bool = true
@export var stop_pause_seconds: float = 3.0
@export var stop_trigger_distance: float = 0.60

var _debug_timer: float = 0.0
var _finished: bool = false
var _final_stop_locked: bool = false
var _curve_length: float = 0.0
var _stop_timer: float = 0.0
var _passed_stop_indices: Dictionary = {}

# World coordinates of stop markers that are close to the current convoy path.
# These come from IntroductionBase.tscn transformed by the KiuPathObj position
# used in convoying.tscn.
var _stop_markers: Array[Vector3] = [
	Vector3(5.732896, 0.08, 3.327136),
	Vector3(7.571691, 0.08, 2.332410),
	Vector3(5.755568, 0.08, 0.866606),
]

@onready var _visual: Node3D = get_node_or_null("LeaderVehicle") as Node3D


func _ready() -> void:
	rotation_mode = PathFollow3D.ROTATION_Y
	loop = false
	add_to_group("npc_leader")
	_refresh_curve_length()
	_apply_visual_offset()
	reset_leader()
	set_process(true)
	print("[KvatiLeaderPath] ready curve_length=", _curve_length, " speed=", speed)


func _refresh_curve_length() -> void:
	var path := get_parent() as Path3D
	if path != null and path.curve != null:
		_curve_length = path.curve.get_baked_length()
	else:
		_curve_length = 0.0


func _get_current_lane_side_offset() -> float:
	if not after_second_turn_right_offset_enabled:
		return lane_side_offset
	if progress <= after_second_turn_offset_start_progress:
		return lane_side_offset
	if after_second_turn_offset_blend_distance <= 0.001:
		return after_second_turn_lane_side_offset

	var t := (progress - after_second_turn_offset_start_progress) / after_second_turn_offset_blend_distance
	t = clamp(t, 0.0, 1.0)
	t = t * t * (3.0 - 2.0 * t)
	return lerp(lane_side_offset, after_second_turn_lane_side_offset, t)


func _apply_visual_offset() -> void:
	if _visual == null:
		return
	_visual.position.x = _get_current_lane_side_offset()
	_visual.rotation.y = deg_to_rad(model_yaw_degrees)


func reset_leader() -> void:
	_refresh_curve_length()
	progress = start_progress
	_finished = false
	_final_stop_locked = false
	_stop_timer = 0.0
	_passed_stop_indices.clear()
	_debug_timer = 999.0
	_apply_visual_offset()
	print("[KvatiLeaderPath] reset progress=", progress, " speed=", speed)


func set_speed(new_speed: float) -> void:
	var requested_speed := float(new_speed)

	if _final_stop_locked and requested_speed > 0.0:
		speed = 0.0
		if debug_print:
			print("[KvatiLeaderPath] final stop locked; ignoring speed=", requested_speed)
		return

	speed = clamp(requested_speed, min_speed, max_speed)
	if speed > 0.0:
		_finished = false
	print("[KvatiLeaderPath] speed set to ", speed)


func _process(delta: float) -> void:
	if _finished:
		return
	if _curve_length <= 0.001:
		_refresh_curve_length()
		if _curve_length <= 0.001:
			return
	if speed <= 0.0:
		return

	if _stop_timer > 0.0:
		_stop_timer -= delta
		if _stop_timer <= 0.0:
			print("[KvatiLeaderPath] stop pause finished")
		return

	_check_stop_marker()
	if _stop_timer > 0.0:
		return

	var effective_speed := speed
	if first_turn_slowdown_enabled and progress >= first_turn_start_progress and progress <= first_turn_end_progress:
		effective_speed = speed * first_turn_speed_multiplier
	elif second_turn_slowdown_enabled and progress >= second_turn_start_progress and progress <= second_turn_end_progress:
		effective_speed = speed * second_turn_speed_multiplier

	progress += effective_speed * delta
	_apply_visual_offset()
	if stop_after_second_turn_enabled and progress >= stop_after_second_turn_progress:
		progress = stop_after_second_turn_progress
		_apply_visual_offset()
		_finished = true
		_final_stop_locked = true
		speed = 0.0
		print("[KvatiLeaderPath] final stop after second turn at progress=", progress)
		return


	if progress >= _curve_length:
		if loop_to_start:
			progress = fposmod(progress, _curve_length)
			_passed_stop_indices.clear()
			print("[KvatiLeaderPath] looped to start")
		else:
			progress = _curve_length
			_finished = true
			speed = 0.0
			print("[KvatiLeaderPath] reached end and stopped")

	_debug_timer += delta
	if debug_print and _debug_timer >= 1.0:
		_debug_timer = 0.0
		print("[KvatiLeaderPath] moving progress=", progress, "/", _curve_length, " speed=", speed, " pos=", global_position)


func _check_stop_marker() -> void:
	if not stop_on_markers:
		return
	for i in range(_stop_markers.size()):
		if _passed_stop_indices.has(i):
			continue
		var marker := _stop_markers[i]
		var dx := global_position.x - marker.x
		var dz := global_position.z - marker.z
		var distance_xz := sqrt(dx * dx + dz * dz)
		if distance_xz <= stop_trigger_distance:
			_passed_stop_indices[i] = true
			_stop_timer = stop_pause_seconds
			print("[KvatiLeaderPath] stop marker ", i, " reached; pausing for ", stop_pause_seconds, "s")
			return
