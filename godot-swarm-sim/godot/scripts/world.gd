extends Node3D
## Spawns N drones from drones.json and manages the shared WakeField.
##
## Attached to the World root node. Reads the fleet config at scene load,
## instances the Drone.tscn scene for each entry, configures the CRSF I/O
## interface child nodes per drone, and calls WakeFieldNode.swap() once per
## physics frame so the double-buffer advances after all drones have stepped.

const DRONE_SCENE := preload("res://scenes/drone.tscn")
const DRONES_JSON_DEFAULT := "res://drones.json"

@onready var wake_field: WakeFieldNode = $WakeField


func _ready() -> void:
	var config := _load_fleet_config()
	if config.is_empty():
		push_warning("drones.json not found or empty; spawning single default drone")
		_spawn_default_drone()
		return

	var prefix_base: String = config.get("prefix_base", "sim")
	var drones_arr: Array = config.get("drones", [])

	if drones_arr.is_empty():
		push_warning("drones.json has no entries; spawning single default drone")
		_spawn_default_drone()
		return

	for entry: Dictionary in drones_arr:
		var drone_id: int = entry.get("id", 0)
		var spawn_pos: Array = entry.get("spawn", [0.0, 1.0, 0.0])
		var is_active: bool = entry.get("active", false)
		var preset_path: String = entry.get("preset", "")
		var prefix_override: String = entry.get("prefix_override", "")

		var drone := DRONE_SCENE.instantiate()
		drone.name = "Drone%d" % drone_id
		drone.position = Vector3(spawn_pos[0], spawn_pos[1], spawn_pos[2])
		drone.set("drone_id", drone_id)

		if preset_path != "":
			var preset_res = load(preset_path)
			if preset_res:
				drone.set("preset", preset_res)

		# Configure ZenohIOInterface topics for this drone.
		var prefix: String
		if prefix_override != "":
			prefix = prefix_override
		else:
			prefix = "%s%d" % [prefix_base, drone_id]
		var zenoh_io := drone.get_node_or_null("ZenohIO")
		if zenoh_io:
			zenoh_io.set("rc_topic", "%s/crsf/rc" % prefix)
			zenoh_io.set("autopilot_topic", "%s/crsf/rc/autopilot" % prefix)
			zenoh_io.set("telemetry_topic", "%s/crsf/telemetry" % prefix)

# Only the active drone gets a GodotInputInterface (keyboard input).
		if is_active:
			var local_io = GodotInputInterface.new()
			local_io.name = "LocalIO"
			drone.add_child(local_io)

		add_child(drone)
		drone.add_to_group("drones")
		print("Spawned Drone%d at %s (active=%s)" % [drone_id, str(drone.position), str(is_active)])

	print("Fleet: %d drones spawned" % drones_arr.size())


func _physics_process(_dt: float) -> void:
	# Advance the wake field's double buffer so next tick's samples
	# see this tick's contributions.
	if wake_field:
		wake_field.swap()


func _spawn_default_drone() -> void:
	var drone := DRONE_SCENE.instantiate()
	drone.name = "Drone0"
	drone.position = Vector3(0, 1, 0)
	drone.set("drone_id", 0)
	# Default drone always gets keyboard input.
	var local_io = GodotInputInterface.new()
	local_io.name = "LocalIO"
	drone.add_child(local_io)
	add_child(drone)
	drone.add_to_group("drones")
	print("Spawned default Drone0")


func _load_fleet_config() -> Dictionary:
	var path: String
	if OS.has_environment("GSS_DRONES_JSON"):
		path = OS.get_environment("GSS_DRONES_JSON")
	else:
		path = DRONES_JSON_DEFAULT
	if not FileAccess.file_exists(path):
		return {}
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return {}
	var text := f.get_as_text()
	f.close()
	var json := JSON.new()
	if json.parse(text) != OK:
		push_error("Failed to parse drones.json: %s" % json.get_error_message())
		return {}
	if json.data is Dictionary:
		return json.data
	return {}
