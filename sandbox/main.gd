extends Node3D
## VoxBox Sandbox. Pick a real place and walk around it.
##
## Command line (after "--"):
##   --places <dir>     folder that holds the built places (default: ../places next to the project)
##   --place <name>     load this place straight away
##   --shots <dir>      take screenshots of the place into <dir> and quit
##   --walk-test        walk every road and path and stand at every door, print a report and quit

const EYE := 1.6
const WALK_SPEED := 4.0
const RUN_SPEED := 9.0
const MAX_CLIMB := 0.45  # highest step up per 0.25 m that can be walked

var places_dir := ""
var place := {}
var heights := PackedFloat32Array()
var world: Node3D
var player: CharacterBody3D
var cam: Camera3D
var ui: CanvasLayer
var menu: Control
var hud: Label
var material: ShaderMaterial
var yaw := 0.0
var pitch := 0.0
var args := {}
var lines_only_close := false


func _ready() -> void:
	var a := OS.get_cmdline_user_args()
	var i := 0
	while i < a.size():
		if a[i] in ["--places", "--place", "--shots"] and i + 1 < a.size():
			args[a[i]] = a[i + 1]
			i += 1
		else:
			args[a[i]] = true
		i += 1
	# built places live in a "places" folder: next to the program when it is a finished build,
	# next to the project folder when run from the Godot editor
	var default_dir := OS.get_executable_path().get_base_dir().path_join("places")
	if OS.has_feature("editor"):
		default_dir = ProjectSettings.globalize_path("res://").path_join("../places").simplify_path()
	places_dir = args.get("--places", default_dir)
	_build_environment()
	_build_ui()
	if args.has("--place"):
		await _load_place(args["--place"])
		if args.has("--shots"):
			await _take_shots(args["--shots"])
			get_tree().quit()
		elif args.has("--walk-test"):
			await _walk_test()
	else:
		_show_menu()


# ---------------------------------------------------------------- environment

func _build_environment() -> void:
	# Lit only by the sun (and the sky it lights).
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-48, -35, 0)
	if args.has("--sun2"):
		sun.rotation_degrees = Vector3(-70, 100, 0)
	sun.light_energy = 0.9
	sun.light_color = Color(1.0, 0.97, 0.9)
	sun.shadow_enabled = not args.has("--no-shadow")
	if args.has("--bias"):
		sun.shadow_bias = 0.3
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS
	sun.directional_shadow_max_distance = 110.0
	if not args.has("--bias"):
		sun.shadow_bias = 0.03
	sun.shadow_normal_bias = 0.0
	sun.directional_shadow_blend_splits = true
	add_child(sun)
	var sky_mat := ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.3, 0.48, 0.78)
	sky_mat.sky_horizon_color = Color(0.72, 0.82, 0.92)
	sky_mat.ground_bottom_color = Color(0.3, 0.32, 0.3)
	sky_mat.ground_horizon_color = Color(0.72, 0.82, 0.92)
	var sky := Sky.new()
	sky.sky_material = sky_mat
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.45
	env.ambient_light_sky_contribution = 0.7
	env.ambient_light_color = Color(0.8, 0.8, 0.8)
	env.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)
	material = ShaderMaterial.new()
	material.shader = load("res://voxel.gdshader")
	material.set_shader_parameter("debug_kind", args.has("--debug-kind"))


# ---------------------------------------------------------------- user interface

func _build_ui() -> void:
	ui = CanvasLayer.new()
	add_child(ui)
	hud = Label.new()
	hud.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
	hud.grow_vertical = Control.GROW_DIRECTION_BEGIN
	hud.position = Vector2(16, -16)
	hud.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.8))
	hud.visible = false
	ui.add_child(hud)


func _show_menu() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	if menu:
		menu.queue_free()
	var view := get_viewport().get_visible_rect().size
	var pad := maxf(16.0, minf(view.x, view.y) * 0.04)
	menu = ColorRect.new()
	(menu as ColorRect).color = Color(0.09, 0.1, 0.12, 0.96)
	menu.set_anchors_preset(Control.PRESET_FULL_RECT)
	ui.add_child(menu)
	var margin := MarginContainer.new()
	margin.set_anchors_preset(Control.PRESET_FULL_RECT)
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, int(pad))
	menu.add_child(margin)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", int(pad * 0.5))
	margin.add_child(box)
	var title := Label.new()
	title.text = "VoxBox Sandbox"
	title.add_theme_font_size_override("font_size", int(clampf(view.y * 0.06, 22, 64)))
	box.add_child(title)
	var sub := Label.new()
	sub.text = "Pick a place to walk around.  W A S D to move, Shift to run, mouse to look, Esc for this menu."
	sub.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	box.add_child(sub)
	var scroll := ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	box.add_child(scroll)
	var list := VBoxContainer.new()
	list.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	list.add_theme_constant_override("separation", int(pad * 0.3))
	scroll.add_child(list)
	var names := _list_places()
	if names.is_empty():
		var none := Label.new()
		none.text = "No built places found in " + places_dir
		list.add_child(none)
	for n in names:
		var b := Button.new()
		b.text = n.replace("_", " ").capitalize()
		b.custom_minimum_size = Vector2(0, clampf(view.y * 0.08, 40, 96))
		b.pressed.connect(func(): _start(n))
		list.add_child(b)
	# Credits are shown at startup, before any place is entered.
	var credits := Label.new()
	credits.text = _credits_text()
	credits.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	credits.add_theme_font_size_override("font_size", int(clampf(view.y * 0.02, 10, 18)))
	credits.modulate = Color(1, 1, 1, 0.7)
	box.add_child(credits)
	if not get_viewport().size_changed.is_connected(_on_resize):
		get_viewport().size_changed.connect(_on_resize)


func _on_resize() -> void:
	if menu and menu.visible:
		_show_menu()


func _list_places() -> Array:
	var out := []
	var d := DirAccess.open(places_dir)
	if d:
		for sub in d.get_directories():
			if FileAccess.file_exists(places_dir.path_join(sub).path_join("place.json")):
				out.append(sub)
	out.sort()
	return out


func _credits_text() -> String:
	for n in _list_places():
		var j = JSON.parse_string(FileAccess.get_file_as_string(places_dir.path_join(n).path_join("place.json")))
		if j and j.has("credits"):
			return j["credits"]
	return "Map data (c) OpenStreetMap contributors."


func _start(n: String) -> void:
	menu.visible = false
	await _load_place(n)
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED


# ---------------------------------------------------------------- loading a place

func _load_place(n: String) -> void:
	if world:
		world.queue_free()
	world = Node3D.new()
	add_child(world)
	var dir: String = places_dir.path_join(n)
	place = JSON.parse_string(FileAccess.get_file_as_string(dir.path_join("place.json")))
	if place == null:
		push_error("cannot read place " + dir)
		get_tree().quit(1)
		return
	# ground colour map
	var gc: Dictionary = place["ground_colours"]
	var img := Image.create_from_data(int(gc["width"]), int(gc["height"]), false, Image.FORMAT_RGB8, FileAccess.get_file_as_bytes(dir.path_join(gc["file"])))
	material.set_shader_parameter("ground_map", ImageTexture.create_from_image(img))
	var mn := Vector2(place["min"][0], place["min"][1])
	var mx := Vector2(place["max"][0], place["max"][1])
	material.set_shader_parameter("ground_min", mn)
	material.set_shader_parameter("ground_size", mx - mn)
	for file in ["surface.vxb", "voxels.vxb"]:
		_load_meshes(dir.path_join(file))
	_load_collision(dir.path_join("collision.vxc"))
	_load_shadow_cores(dir.path_join("shadow.vxc"))
	_load_props()
	var h: Dictionary = place["heights"]
	heights = FileAccess.get_file_as_bytes(dir.path_join(h["file"])).to_float32_array()
	_spawn_player()
	hud.text = "%s    %d buildings, %d windows, %d doors" % [n.replace("_", " ").capitalize(), place["stats"]["buildings"], place["stats"]["windows"], place["stats"]["doors"]]
	hud.visible = not args.has("--shots")
	await get_tree().physics_frame
	await get_tree().physics_frame


func _load_meshes(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		push_error("missing " + path)
		return
	f.get_buffer(4)
	var count := f.get_32()
	for _n in count:
		var name_len := f.get_32()
		var mesh_name := f.get_buffer(name_len).get_string_from_utf8()
		var nv := f.get_32()
		var ni := f.get_32()
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = f.get_buffer(nv * 12).to_vector3_array()
		arrays[Mesh.ARRAY_TEX_UV] = f.get_buffer(nv * 8).to_vector2_array()
		arrays[Mesh.ARRAY_INDEX] = f.get_buffer(ni * 4).to_int32_array()
		var m := ArrayMesh.new()
		m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		var mi := MeshInstance3D.new()
		mi.name = mesh_name
		mi.mesh = m
		mi.material_override = material
		if args.has("--vox-noshadow") and mesh_name.begins_with("vox"):
			mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			mi.visible = not args.has("--show-cores")
		world.add_child(mi)


func _load_collision(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return
	f.get_buffer(4)
	var count := f.get_32()
	var body := StaticBody3D.new()
	body.name = "Solid"
	world.add_child(body)
	for _n in count:
		var name_len := f.get_32()
		f.get_buffer(name_len)
		var ni := f.get_32()
		var shape := ConcavePolygonShape3D.new()
		shape.backface_collision = true
		shape.set_faces(f.get_buffer(ni * 12).to_vector3_array())
		var cs := CollisionShape3D.new()
		cs.shape = shape
		body.add_child(cs)


## Solid insides of the buildings: never drawn, they only cast shadows, so no sunlight can slip
## along a wall and light up its foot.
func _load_shadow_cores(path: String) -> void:
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		return
	f.get_buffer(4)
	var count := f.get_32()
	for _n in count:
		var name_len := f.get_32()
		f.get_buffer(name_len)
		var ni := f.get_32()
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = f.get_buffer(ni * 12).to_vector3_array()
		var m := ArrayMesh.new()
		m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		var mat := StandardMaterial3D.new()
		mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		var mi := MeshInstance3D.new()
		mi.name = "ShadowCores"
		mi.mesh = m
		mi.material_override = mat
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON if args.has("--show-cores") else GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY
		print("cores: ", ni / 3, " triangles, bounds ", m.get_aabb())
		world.add_child(mi)


func _prop_basis(p: Dictionary) -> Basis:
	var yw: float = p["yaw"]
	var s: float = p["scale"]
	var x := Vector3(cos(yw), 0, sin(yw))
	var up := Vector3(-p["slope"][0], 1.0, -p["slope"][1]).normalized()
	x = (x - up * up.dot(x)).normalized()
	var z := x.cross(up)
	return Basis(x * s, up * s, z * s)


func _load_props() -> void:
	# One mesh per model and body colour, drawn many times.
	var groups := {}
	for p in place["props"]:
		var key: String = p["model"] + str(p["body"])
		if not groups.has(key):
			groups[key] = []
		groups[key].append(p)
	var body := StaticBody3D.new()
	body.name = "Props"
	world.add_child(body)
	for key in groups:
		var first: Dictionary = groups[key][0]
		var model: Dictionary = place["models"][first["model"]]
		var st := SurfaceTool.new()
		st.begin(Mesh.PRIMITIVE_TRIANGLES)
		for b in model["boxes"]:
			var c = b["colour"] if b["colour"] != null else first["body"]
			var col := Color(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0, 1.0)
			_add_box(st, Vector3(b["min"][0], b["min"][1], b["min"][2]), Vector3(b["max"][0], b["max"][1], b["max"][2]), col)
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.mesh = st.commit()
		mm.instance_count = groups[key].size()
		for k in groups[key].size():
			var p: Dictionary = groups[key][k]
			var t := Transform3D(_prop_basis(p), Vector3(p["pos"][0], p["pos"][1], p["pos"][2]))
			mm.set_instance_transform(k, t)
			# things you can walk into: a box around the foot of each prop
			var cs := CollisionShape3D.new()
			var box := BoxShape3D.new()
			var foot: Array = model["foot"]
			box.size = Vector3(foot[0] * 2.0, 1.2, foot[1] * 2.0) * float(p["scale"])
			cs.shape = box
			cs.transform = Transform3D(_prop_basis(p).orthonormalized(), t.origin + Vector3(0, 0.6, 0))
			body.add_child(cs)
		var mmi := MultiMeshInstance3D.new()
		mmi.multimesh = mm
		mmi.material_override = material
		world.add_child(mmi)


func _add_box(st: SurfaceTool, a: Vector3, b: Vector3, col: Color) -> void:
	var c := [Vector3(a.x, a.y, a.z), Vector3(b.x, a.y, a.z), Vector3(b.x, b.y, a.z), Vector3(a.x, b.y, a.z),
		Vector3(a.x, a.y, b.z), Vector3(b.x, a.y, b.z), Vector3(b.x, b.y, b.z), Vector3(a.x, b.y, b.z)]
	for q in [[0, 1, 2, 3], [5, 4, 7, 6], [4, 0, 3, 7], [1, 5, 6, 2], [3, 2, 6, 7], [4, 5, 1, 0]]:
		for k in [0, 1, 2, 0, 2, 3]:
			st.set_uv(Vector2(roundf(col.r * 255.0) + 256.0 * roundf(col.g * 255.0), roundf(col.b * 255.0) + 256.0 * 255.0))
			st.add_vertex(c[q[k]])


# ---------------------------------------------------------------- the ground and the walker

## Ground height at a point: the engine's smooth walking surface.
func ground(x: float, z: float) -> float:
	var h: Dictionary = place["heights"]
	var nx := int(h["nx"])
	var nz := int(h["nz"])
	var fx := clampf((x - float(place["min"][0])) / float(h["step"]), 0.0, nx - 1.001)
	var fz := clampf((z - float(place["min"][1])) / float(h["step"]), 0.0, nz - 1.001)
	var ix := int(fx)
	var iz := int(fz)
	var tx := fx - ix
	var tz := fz - iz
	var a := lerpf(heights[iz * nx + ix], heights[iz * nx + ix + 1], tx)
	var b := lerpf(heights[(iz + 1) * nx + ix], heights[(iz + 1) * nx + ix + 1], tx)
	return lerpf(a, b, tz)


func _inside(x: float, z: float) -> bool:
	return x > float(place["min"][0]) + 0.5 and x < float(place["max"][0]) - 0.5 and z > float(place["min"][1]) + 0.5 and z < float(place["max"][1]) - 0.5


func _spawn_player() -> void:
	player = CharacterBody3D.new()
	player.motion_mode = CharacterBody3D.MOTION_MODE_FLOATING
	var cs := CollisionShape3D.new()
	var cap := CapsuleShape3D.new()
	cap.radius = 0.3
	cap.height = 1.3
	cs.shape = cap
	# the capsule floats 0.45 m above the feet: kerbs and door steps pass under it
	cs.position = Vector3(0, 0.45 + 0.65, 0)
	player.add_child(cs)
	cam = Camera3D.new()
	cam.position = Vector3(0, EYE, 0)
	cam.near = 0.08
	cam.far = 900.0
	cam.fov = 75.0
	player.add_child(cam)
	world.add_child(player)
	var s: Array = place["spawn"]
	player.position = Vector3(s[0], ground(s[0], s[2]), s[2])
	cam.current = true


## Moves the walker by a horizontal step. Walls stop it (it slides along them); the feet follow the
## ground. Returns the distance actually covered.
func walk_step(motion: Vector3) -> float:
	var from := player.position
	var target := from + motion
	if not _inside(target.x, target.z):
		return 0.0
	# too steep to climb (a retaining wall): no step
	if ground(target.x, target.z) - from.y > MAX_CLIMB * maxf(1.0, motion.length() / 0.25):
		return 0.0
	var col := player.move_and_collide(motion)
	if col:
		var rest := col.get_remainder().slide(col.get_normal())
		rest.y = 0.0
		if rest.length() > 0.001:
			player.move_and_collide(rest)
	player.position.y = ground(player.position.x, player.position.z)
	return Vector2(player.position.x - from.x, player.position.z - from.z).length()


func _physics_process(delta: float) -> void:
	if player == null or args.has("--shots") or args.has("--walk-test") or (menu and menu.visible):
		return
	var dir := Vector3.ZERO
	var fwd := Vector3(-sin(yaw), 0, -cos(yaw))
	var right := Vector3(cos(yaw), 0, -sin(yaw))
	if Input.is_action_pressed("move_forward"):
		dir += fwd
	if Input.is_action_pressed("move_back"):
		dir -= fwd
	if Input.is_action_pressed("move_right"):
		dir += right
	if Input.is_action_pressed("move_left"):
		dir -= right
	if dir.length() > 0.0:
		var speed := RUN_SPEED if Input.is_action_pressed("run") else WALK_SPEED
		walk_step(dir.normalized() * speed * delta)


func _unhandled_input(e: InputEvent) -> void:
	if player == null:
		return
	if e is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		yaw -= e.relative.x * 0.0025
		pitch = clampf(pitch - e.relative.y * 0.0025, -1.5, 1.5)
		player.rotation.y = yaw
		cam.rotation.x = pitch
	elif e is InputEventKey and e.pressed and e.keycode == KEY_ESCAPE:
		_show_menu()
	elif e is InputEventMouseButton and e.pressed and Input.mouse_mode != Input.MOUSE_MODE_CAPTURED and not (menu and menu.visible):
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED


# ---------------------------------------------------------------- screenshots

func _shot(dir: String, n: String, from: Vector3, to: Vector3) -> void:
	player.position = Vector3(from.x, from.y - EYE, from.z)
	player.rotation = Vector3.ZERO
	cam.rotation = Vector3.ZERO
	cam.look_at(to)
	for _k in 4:
		await get_tree().process_frame
	await RenderingServer.frame_post_draw
	get_viewport().get_texture().get_image().save_png(dir.path_join(place["name"] + "_" + n + ".png"))
	print("shot ", n)


func _take_shots(dir: String) -> void:
	DirAccess.make_dir_recursive_absolute(dir)
	var mn := Vector2(place["min"][0], place["min"][1])
	var mx := Vector2(place["max"][0], place["max"][1])
	var mid := (mn + mx) * 0.5
	var gm := ground(mid.x, mid.y)
	if args.has("--only-close"):
		lines_only_close = true
	if not args.has("--only-close"):
		await _shot(dir, "1_overview", Vector3(mid.x - 150, gm + 170, mid.y + 150), Vector3(mid.x, gm, mid.y))
	if not lines_only_close:
		await _shot(dir, "2_closer", Vector3(mid.x + 60, gm + 55, mid.y + 60), Vector3(mid.x, gm + 5, mid.y))
	# street level: along the travel lines, at the spawn, at the steepest stretch and at the tightest one
	var lines: Array = [] if lines_only_close else place["travel_lines"]
	var best_steep := {}
	var best_tight := {}
	var best_mid := {}
	for l in lines:
		var pts: Array = l["points"]
		for k in range(4, pts.size() - 8):
			var a: Array = pts[k]
			var b: Array = pts[k + 6]
			var run := Vector2(b[0] - a[0], b[2] - a[2]).length()
			if run < 3.0 or not _inside(a[0], a[2]) or absf(a[0] - mid.x) > 100 or absf(a[2] - mid.y) > 100:
				continue
			var grade: float = absf(b[1] - a[1]) / run
			var view := {"a": a, "b": b, "uphill": b[1] > a[1]}
			if best_steep.is_empty() or grade > best_steep["v"]:
				best_steep = view.merged({"v": grade})
			if l["kind"] == "road" and (best_tight.is_empty() or a[3] < best_tight["v"]):
				best_tight = view.merged({"v": a[3]})
			var dm := Vector2(a[0] - mid.x, a[2] - mid.y).length()
			if l["kind"] == "road" and (best_mid.is_empty() or dm < best_mid["v"]):
				best_mid = view.merged({"v": dm})
	var k := 3
	for v in [["street", best_mid], ["steepest", best_steep], ["tightest", best_tight]]:
		if v[1].is_empty():
			continue
		var a: Array = v[1]["a"]
		var b: Array = v[1]["b"]
		if not v[1]["uphill"]:
			var t := a
			a = b
			b = t
		var d := Vector3(b[0] - a[0], 0, b[2] - a[2]).normalized()
		var eye := Vector3(a[0], ground(a[0], a[2]) + EYE, a[2])
		await _shot(dir, "%d_%s" % [k, v[0]], eye, eye + d * 10.0 + Vector3(0, (b[1] - a[1]) * 1.2, 0))
		await _shot(dir, "%d_%s_back" % [k, v[0]], eye + d * 6.0 + Vector3(0, b[1] - a[1], 0), eye - d * 6.0)
		k += 1
	# close-ups of where walls meet the ground, at the doors nearest to the middle
	var near_doors: Array = place["doors"].duplicate()
	near_doors.sort_custom(func(p, q): return Vector2(p["pos"][0] - mid.x, p["pos"][2] - mid.y).length() < Vector2(q["pos"][0] - mid.x, q["pos"][2] - mid.y).length())
	for n in mini(2, near_doors.size()):
		var d: Dictionary = near_doors[n]
		var p := Vector3(d["pos"][0], d["pos"][1], d["pos"][2])
		var out := Vector3(d["out"][0], 0, d["out"][1])
		var side := Vector3(-out.z, 0, out.x)
		var e := p + out * 1.6 + side * 0.4
		e.y = ground(e.x, e.z) + 0.7
		await _shot(dir, "7_wallbase%d" % (n + 1), e, p + side * 2.0 + Vector3(0, -0.1, 0))
		var e2 := p + out * 0.45 + side * 0.9
		e2.y = ground(e2.x, e2.z) + 0.3
		await _shot(dir, "8_macro%d" % (n + 1), e2, p + side * 1.8 + Vector3(0, -0.05, 0))
	# doors: the three nearest to the middle
	var doors: Array = place["doors"].duplicate()
	doors.sort_custom(func(p, q): return Vector2(p["pos"][0] - mid.x, p["pos"][2] - mid.y).length() < Vector2(q["pos"][0] - mid.x, q["pos"][2] - mid.y).length())
	for n in mini(0 if lines_only_close else 3, doors.size()):
		var d: Dictionary = doors[n]
		var p := Vector3(d["pos"][0], d["pos"][1], d["pos"][2])
		var out := Vector3(d["out"][0], 0, d["out"][1])
		var side := Vector3(-out.z, 0, out.x)
		var e := p + out * 6.0 + side * 2.5
		e.y = ground(e.x, e.z) + EYE
		await _shot(dir, "%d_door%d" % [k, n + 1], e, p + Vector3(0, 1.3, 0))
	k += 1


# ---------------------------------------------------------------- walk test

## Walks every road and path of the place and stands in front of every door,
## with the same step function the player uses.
func _walk_test() -> void:
	await get_tree().physics_frame
	var stuck := []
	var walked := 0.0
	var lines_done := 0
	var off_ground := 0
	var marked := []
	for b in place["barriers"]:
		marked.append(Vector2(b["pos"][0], b["pos"][1]))
	for l in place["travel_lines"]:
		var pts: Array = l["points"]
		if pts.size() < 3:
			continue
		lines_done += 1
		var started := false
		for k in range(0, pts.size()):
			var goal := Vector2(pts[k][0], pts[k][2])
			if not _inside(goal.x, goal.y):
				# the line leaves the place here; pick it up again where it comes back in
				started = false
				continue
			if not started:
				player.position = Vector3(goal.x, ground(goal.x, goal.y), goal.y)
				started = true
				continue
			var tries := 0
			while Vector2(player.position.x, player.position.z).distance_to(goal) > 0.3 and tries < 40:
				var to := goal - Vector2(player.position.x, player.position.z)
				var step := to.normalized() * minf(0.25, to.length())
				walked += walk_step(Vector3(step.x, 0, step.y))
				tries += 1
			var here := Vector2(player.position.x, player.position.z)
			if here.distance_to(goal) > 0.6:
				# a real barrier the map puts on the path (bollard, gate) is reported by the engine: go round
				var real := false
				for m in marked:
					if m.distance_to(goal) < 2.5 or m.distance_to(here) < 2.5:
						real = true
				if not real:
					stuck.append({"line": l["way"], "class": l["class"], "at": [snappedf(here.x, 0.1), snappedf(here.y, 0.1)], "goal": [snappedf(goal.x, 0.1), snappedf(goal.y, 0.1)]})
				player.position = Vector3(goal.x, ground(goal.x, goal.y), goal.y)
			if absf(player.position.y - float(pts[k][1])) > 0.25 and here.distance_to(goal) <= 0.3:
				off_ground += 1
	# doors: there is room to stand right in front of each one
	var doors_ok := 0
	var doors_bad := []
	for d in place["doors"]:
		var p := Vector2(d["pos"][0], d["pos"][2]) + Vector2(d["out"][0], d["out"][1]) * 0.55
		if not _inside(p.x, p.y):
			continue
		player.position = Vector3(p.x, ground(p.x, p.y), p.y)
		var blocked := player.test_move(player.global_transform, Vector3(0, 0.01, 0))
		var step_up := float(d["pos"][1]) - player.position.y
		if blocked or step_up > 0.35 or step_up < -0.05:
			doors_bad.append({"building": d["building"], "blocked": blocked, "step": snappedf(step_up, 0.01)})
		else:
			doors_ok += 1
	# control: walking straight at a door must be stopped by the building (otherwise this test proves nothing)
	var walls_hold := 0
	var walls_leak := 0
	for d in place["doors"]:
		var dp := Vector2(d["pos"][0], d["pos"][2])
		var o := Vector2(d["out"][0], d["out"][1])
		var s0 := dp + o * 0.9
		if not _inside(s0.x, s0.y) or not _inside(dp.x - o.x * 3.0, dp.y - o.y * 3.0):
			continue
		player.position = Vector3(s0.x, ground(s0.x, s0.y), s0.y)
		for _n in 14:
			walk_step(Vector3(-o.x, 0, -o.y) * 0.2)
		var through := (Vector2(player.position.x, player.position.z) - dp).dot(o)
		if through < -0.15:
			walls_leak += 1
		else:
			walls_hold += 1
	var report := {"place": place["name"], "walls_hold": walls_hold, "walls_leak": walls_leak, "lines": lines_done, "metres_walked": int(walked), "stuck": stuck.size(), "height_mismatch": off_ground,
		"doors_ok": doors_ok, "doors_bad": doors_bad.size(), "stuck_first": stuck.slice(0, 5), "doors_bad_first": doors_bad.slice(0, 5)}
	print("WALKTEST ", JSON.stringify(report))
	get_tree().quit(0 if stuck.is_empty() and doors_bad.is_empty() and off_ground == 0 and walls_leak == 0 else 1)
