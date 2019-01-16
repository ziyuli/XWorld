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
uniform sampler2D blend_map_0;
uniform sampler2D blend_map_1;
uniform sampler2DArray terrain_maps;
uniform int terrain_size;
uniform int span_size = 20;
uniform int height_map_size = 128;
uniform int texture_map_size = 128;
uniform float terrain_scale[8];

vec3 terrain_normal() {
	vec2 pos;
	pos.x = fs_in.tc.x / span_size * terrain_size;
	pos.y = fs_in.tc.y / span_size * terrain_size;
	return -normalize(texture(normal_map, pos).xyz);
}

vec3 terrain_texture(float id) {
	vec2 pos;
	pos.x = fs_in.tc.x / span_size * terrain_size;
	pos.y = fs_in.tc.y / span_size * terrain_size;
	return texture2DArray(terrain_maps, vec3(pos * terrain_scale[int(id)], id)).rgb;
}

void main() {

	vec2 pos;
	pos.x = fs_in.tc.x / span_size * terrain_size;
	pos.y = fs_in.tc.y / span_size * terrain_size;
	vec4 blend_0 = texture(blend_map_0, pos);
	vec4 blend_1 = texture(blend_map_1, pos);

	vec3 layers[8];
	layers[0] = terrain_texture(0) * blend_0.x;
	layers[1] = terrain_texture(1) * blend_0.y;
	layers[2] = terrain_texture(2) * blend_0.z;
	layers[3] = terrain_texture(3) * blend_0.w;
	layers[4] = terrain_texture(4) * blend_1.x;
	layers[5] = terrain_texture(5) * blend_1.y;
	layers[6] = terrain_texture(6) * blend_1.z;
	layers[7] = terrain_texture(7) * blend_1.w;

	vec3 albedo = vec3(0,0,0);
	for (int i = 0; i < 8; ++i) {
		albedo += layers[i];
	}

	albedo /= dot(blend_0, vec4(1)) + dot(blend_1, vec4(1));

	vec3 normal = terrain_normal();
	vec3 light  = normalize(light_directional);
	vec3 diffuse = max(0.0, dot(normal, light)) * vec3(0.9) * albedo;
	vec3 ambient = vec3(0.1) * albedo;

	FragColor = diffuse + ambient;
}