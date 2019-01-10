#version 410 core

precision highp float;

uniform int terrain_size;
uniform int chunk_size;

out VS_OUT {
	vec2 tc;
	int instance_id;
} vs_out;

void main() {
	const vec3 quad[] = vec3[](vec3(1.0, 0.0, 0.0),
							   vec3(1.0, 0.0, 1.0),
							   vec3(0.0, 0.0, 0.0),
							   vec3(0.0, 0.0, 1.0));

	int chunks_per_side = terrain_size / chunk_size;

	vec2 offset;
	offset.x = (gl_InstanceID % chunks_per_side) * chunk_size;
	offset.y = (gl_InstanceID / chunks_per_side) * chunk_size;

	vec2 pos = quad[gl_VertexID].xz * chunk_size + offset;

	vs_out.tc = pos / vec2(terrain_size, terrain_size);
	vs_out.instance_id = gl_InstanceID;

	gl_Position = vec4(pos.x, 0.0, pos.y, 1.0);
}