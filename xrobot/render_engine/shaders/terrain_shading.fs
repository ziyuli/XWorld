#version 330 core

out vec3 FragColor;
noperspective in vec2 TexCoords;

uniform sampler2D height_map;
// uniform sampler2DArray textures;
// uniform lowp int textures_size;

uniform float ambient = 0.1;
uniform vec3 dlight = vec3(1, 2, 3);

const ivec3 off = ivec3(-1,0,1);
const vec2 size = vec2(2.0,0.0);

vec4 normal() {
    vec4 height = texture(height_map, TexCoords);
    float s11 = height.x;
    float s01 = textureOffset(height_map, TexCoords, off.xy).x;
    float s21 = textureOffset(height_map, TexCoords, off.zy).x;
    float s10 = textureOffset(height_map, TexCoords, off.yx).x;
    float s12 = textureOffset(height_map, TexCoords, off.yz).x;
    vec3 grad_a = normalize(vec3(size.xy,s21-s01));
    vec3 grad_b = normalize(vec3(size.yx,s12-s10));
    return vec4(cross(grad_a,grad_b),height);
}

void main() {
	vec4  normalHeight = normal();
	float height = normalHeight.w;
	float diffuse = max(0.0, dot(normalHeight.xyz, normalize(dlight)));
	float irradiance = ambient + diffuse;
	FragColor = vec3(irradiance);
}