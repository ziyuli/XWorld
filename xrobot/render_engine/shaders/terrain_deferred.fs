#version 410 core
#extension GL_EXT_texture_array : enable

layout (location = 0) out vec4 gPosition;
layout (location = 1) out vec4 gNormal;
layout (location = 2) out vec4 gAlbedo;

in TES_OUT {
	vec2 tc;
	vec3 world;
	vec3 eye;
} fs_in;

uniform mat4 view;
uniform mat4 projection;
uniform sampler2D normal_map;
uniform sampler2D texture_id_map;
uniform sampler2DArray terrain_maps;
uniform int terrain_size;
uniform int span_size = 20;
uniform int height_map_size = 128;
uniform int texture_map_size = 128;

int tcms = (height_map_size - 1) * texture_map_size + 1;
int centerIndex = int(fs_in.tc.x * tcms) + int(fs_in.tc.y * tcms) * tcms;
vec4 parts[9];
vec4 mixes[3];

vec3 normal() {
	vec2 pos;
	pos.x = fs_in.tc.x / span_size * terrain_size;
	pos.y = fs_in.tc.y / span_size * terrain_size;
	return -normalize(texture(normal_map, pos).xyz);
}

vec3 terrain(float id) {
	vec2 pos;
	pos.x = fs_in.tc.x / span_size * terrain_size;
	pos.y = fs_in.tc.y / span_size * terrain_size;
	return texture2DArray(terrain_maps, vec3(pos * 5, id)).rgb;
}

float texture_id() {
	vec2 pos;
	pos.x = fs_in.tc.x / span_size * terrain_size;
	pos.y = fs_in.tc.y / span_size * terrain_size;
	return texture(texture_id_map, pos).x;
}

void main() {

	float id = texture_id();
	float id0 = floor(id);
	float id1 = ceil(id);

	gAlbedo = vec4(mix(terrain(id0), terrain(id1), id), 0.0);
	gPosition = vec4(fs_in.eye, 0.7);
	gNormal.xyz = normal();
	gNormal.a = 1.0;
}