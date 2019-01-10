#version 410 core
#extension GL_EXT_texture_array : enable

out vec3 FragColor;

in TES_OUT {
	vec2 tc;
	vec3 world;
	vec3 eye;
} fs_in;

// Light
uniform vec3 light_directional = -vec3(1, 2, 3);

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

#define TOP_LEFT		0
#define TOP				1
#define TOP_RIGHT		2
#define LEFT			3
#define CENTER			4
#define RIGHT			5
#define BOTTOM_LEFT		6
#define BOTTOM			7
#define BOTTOM_RIGHT	8

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

	vec3 albedo = mix(terrain(id0), terrain(id1), id);
	vec3 normal = normal();
	vec3 light  = normalize(light_directional);
	vec3 diffuse = max(0.0, dot(normal, light)) * vec3(0.9) * albedo;
	vec3 ambient = vec3(0.1) * albedo;

	FragColor = diffuse + ambient;
}