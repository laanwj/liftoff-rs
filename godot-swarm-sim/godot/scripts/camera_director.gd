extends Control
## CameraDirector: manages single-drone and tiled multi-drone viewports.
##
## Placed as a child of a full-screen Control node. Creates SubViewport
## children dynamically for each drone's FPV camera, and arranges them
## in either single-active (one full-screen) or tiled (√N×√N grid) mode.
##
## Controls:
##   F1       — toggle between single and tiled mode
##   0..9     — set active drone (single mode: shows that drone's FPV;
##              multi mode: highlights and routes manual RC to that drone)
##   Shift+N  — set active drone ID = 10 + N
##
## The director discovers drones by querying the World node's children
## for nodes in the "drones" group (world.gd adds them to the group
## on spawn).

enum ViewMode { SINGLE, TILED }

var view_mode: ViewMode = ViewMode.SINGLE
var active_drone_id: int = 0
var drones: Array[Node] = []

# SubViewportContainers, one per drone, created dynamically.
var viewport_containers: Array[SubViewportContainer] = []
var viewports: Array[SubViewport] = []


func _ready() -> void:
	# Wait one frame for world.gd to spawn drones and add them to the group.
	await get_tree().process_frame
	_discover_drones()
	_build_viewports()
	_apply_layout()


func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		var key := (event as InputEventKey)
		if key.keycode == KEY_F1:
			_toggle_mode()
		elif key.keycode >= KEY_0 and key.keycode <= KEY_9:
			var digit := key.keycode - KEY_0
			var id := digit + (10 if key.shift_pressed else 0)
			_set_active(id)


func _toggle_mode() -> void:
	if view_mode == ViewMode.SINGLE:
		view_mode = ViewMode.TILED
	else:
		view_mode = ViewMode.SINGLE
	_apply_layout()
	print("CameraDirector: mode = %s" % ("TILED" if view_mode == ViewMode.TILED else "SINGLE"))


func _set_active(id: int) -> void:
	if id == active_drone_id:
		return
	active_drone_id = id
	_update_active_on_drones()
	_apply_layout()
	print("CameraDirector: active drone = %d" % id)


func _discover_drones() -> void:
	drones.clear()
	var world := get_tree().current_scene
	if world == null:
		return
	for child in world.get_children():
		if child.is_in_group("drones"):
			drones.append(child)
	# Sort by drone_id for deterministic ordering.
	drones.sort_custom(func(a, b): return a.get("drone_id") < b.get("drone_id"))
	print("CameraDirector: discovered %d drone(s)" % drones.size())


func _build_viewports() -> void:
	# Clear any previous viewports.
	for c in viewport_containers:
		c.queue_free()
	viewport_containers.clear()
	viewports.clear()

	for i in range(drones.size()):
		var drone := drones[i]
		var container := SubViewportContainer.new()
		container.stretch = true
		container.name = "VPC_%d" % i
		add_child(container)

		var vp := SubViewport.new()
		vp.name = "VP_%d" % i
		vp.render_target_update_mode = SubViewport.UPDATE_ALWAYS
		vp.handle_input_locally = false
		container.add_child(vp)

		# Reparent the drone's FPV camera into this SubViewport.
		# We use a RemoteTransform3D so the camera stays in the
		# drone's coordinate space but renders into the SubViewport.
		# Actually, simpler: create a new Camera3D in the SubViewport
		# that follows the drone's FPV camera via remote_path.
		var cam := Camera3D.new()
		cam.name = "FPVCam_%d" % i
		cam.fov = 120.0
		vp.add_child(cam)

		# We'll sync the camera transform every frame in _process.
		viewport_containers.append(container)
		viewports.append(vp)


func _process(_dt: float) -> void:
	# Sync each SubViewport camera to its drone's FPV camera transform.
	for i in range(min(drones.size(), viewports.size())):
		var drone := drones[i]
		var fpv_path: NodePath = drone.get("fpv_camera_path")
		if fpv_path.is_empty():
			continue
		var fpv_cam := drone.get_node_or_null(fpv_path)
		if fpv_cam == null:
			continue
		var vp_cam: Camera3D = viewports[i].get_node_or_null("FPVCam_%d" % i)
		if vp_cam == null:
			continue
		vp_cam.global_transform = fpv_cam.global_transform
		vp_cam.fov = fpv_cam.fov


func _apply_layout() -> void:
	if drones.is_empty():
		return

	var screen_size := get_viewport_rect().size
	if screen_size.x < 1 or screen_size.y < 1:
		# Headless / zero-size viewport — skip layout to avoid division by zero.
		screen_size = Vector2(1920, 1080)

	match view_mode:
		ViewMode.SINGLE:
			for i in range(viewport_containers.size()):
				var c := viewport_containers[i]
				var is_active := _drone_id_at(i) == active_drone_id
				c.visible = is_active
				if is_active:
					c.position = Vector2.ZERO
					c.size = screen_size
		ViewMode.TILED:
			var n := viewport_containers.size()
			var cols := ceili(sqrt(float(n)))
			var rows := ceili(float(n) / float(cols))
			var tile_w := screen_size.x / cols
			var tile_h := screen_size.y / rows
			for i in range(n):
				var c := viewport_containers[i]
				c.visible = true
				var col := i % cols
				var row := i / cols
				c.position = Vector2(col * tile_w, row * tile_h)
				c.size = Vector2(tile_w, tile_h)


func _drone_id_at(index: int) -> int:
	if index < 0 or index >= drones.size():
		return -1
	return drones[index].get("drone_id")


func _update_active_on_drones() -> void:
	pass
