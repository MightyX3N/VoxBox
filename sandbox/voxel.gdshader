// One shader for everything VoxBox draws: voxels, roofs, ground.
// Faces are flat shaded from their real geometry, and every 10 cm cell gets a
// slightly different brightness so surfaces read as small voxels.
shader_type spatial;
render_mode cull_disabled;

uniform sampler2D ground_map : filter_nearest, repeat_disable;
uniform vec2 ground_min;
uniform vec2 ground_size;
uniform float jitter = 0.045;
uniform bool debug_kind = false;

varying vec3 wpos;
varying vec4 vcol;

float hash(vec3 p) {
	p = fract(p * 0.3183099 + vec3(0.11, 0.17, 0.13));
	p *= 17.0;
	return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

void vertex() {
	wpos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	// colour is packed into the two UV numbers: r + 256 g, b + 256 a
	vec2 hi = floor(UV / 256.0);
	vcol = vec4(UV.x - hi.x * 256.0, hi.x, UV.y - hi.y * 256.0, hi.y) / 255.0;
}

void fragment() {
	vec3 n = normalize(cross(dFdy(wpos), dFdx(wpos)));
	vec3 to_cam = CAMERA_POSITION_WORLD - wpos;
	if (dot(n, to_cam) < 0.0) {
		n = -n;
	}
	vec3 col = vcol.rgb;
	bool top_like = false;
	if (vcol.a < 0.503) {
		// ground: colour of the 10 cm cell under the point
		vec2 p = wpos.xz;
		if (vcol.a > 0.4) {
			// a real step (kerb, bank): colour of the higher cell behind the face, and lit evenly
			// whichever way the step faces, so a diagonal kerb reads as one kerb
			vec2 h = n.xz;
			p -= (length(h) > 0.001 ? normalize(h) : vec2(0.0)) * 0.05;
			n = normalize(vec3(n.x * 0.15, 1.0, n.z * 0.15));
		} else if (vcol.a > 0.1) {
			// a sliver that only closes the surface between two patches: looks like the ground itself
			top_like = true;
		}
		col = texture(ground_map, (p - ground_min) / ground_size).rgb;
	} else if (vcol.a > 0.75 && vcol.a < 0.8) {
		top_like = true;  // the same kind of sliver on a roof
	}
	if (top_like) {
		n = vec3(0.0, 1.0, 0.0);
	}
	vec3 cell = floor((wpos - n * 0.05) / 0.1);
	if (vcol.a > 0.503 && vcol.a < 0.998 && !top_like && abs(n.y) < 0.5) {
		// a wall is lit as the flat wall it is, not as a staircase of voxel sides
		float ang = (vcol.a * 255.0 - 129.0) / 126.0 * 6.2831853;
		vec3 wall = vec3(cos(ang), 0.0, sin(ang));
		if (dot(wall, n) > -0.2) {
			n = wall;
		}
	}
	col *= 1.0 + (hash(cell) - 0.5) * jitter * 2.0;
	if (debug_kind) {
		// what kind of face is this? (for finding drawing faults)
		col = vcol.a < 0.1 ? vec3(1.0, 0.0, 0.0) : vcol.a < 0.4 ? vec3(0.0, 1.0, 0.0) : vcol.a < 0.503 ? vec3(0.0, 0.0, 1.0) : vcol.a < 0.75 ? vec3(1.0, 1.0, 0.0) : vcol.a < 0.8 ? vec3(0.0, 1.0, 1.0) : vcol.a < 0.998 ? vec3(1.0, 0.6, 0.0) : vec3(1.0, 0.0, 1.0);
	}
	ALBEDO = col;
	ROUGHNESS = 0.95;
	SPECULAR = 0.1;
	NORMAL = normalize((VIEW_MATRIX * vec4(n, 0.0)).xyz);
}
