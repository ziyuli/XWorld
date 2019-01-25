#version 330 core

out float FragColorLayer;

flat in int LayerGS;
in vec2 TexCoordsGS;

uniform samplerCube tex;
uniform float upper = 0.85;
uniform float lower = 0.15;
uniform float near = 0.02;
uniform float far = 20.0;

const float fov   = 1.5707962;
const float fov_2 = 0.7853982;
const float tan_fov_2 = tan(fov_2);

float linearize(float depth_lg) {
    return (2 * near) / (far + near - depth_lg * (far - near));
}

float spherical_depth(float depth_linear, float phi, float theta) {
	return depth_linear / abs(cos(phi) * cos(theta));
}

void main() {

	if(TexCoordsGS.y < lower || TexCoordsGS.y > upper)
		discard;
	
	vec2 map_coord = 2.0 * TexCoordsGS - 1.0;

	vec2 phi_theta = fov_2 * map_coord; 
	vec2 tan_phi_theta = tan(phi_theta);

	map_coord.x = tan_phi_theta.x / tan_fov_2;
	map_coord.y = tan_phi_theta.y / (cos(phi_theta.x) * tan_fov_2);

	vec3 sample_direction = vec3(0.0);
	if(LayerGS == 0)
		sample_direction = vec3(1.0, map_coord.y, map_coord.x);
	else if(LayerGS == 1)
		sample_direction = vec3(-1.0, map_coord.y, -map_coord.x);
	else if(LayerGS == 2)
		sample_direction = vec3(map_coord.x, 1.0, map_coord.y);
	else if(LayerGS == 3)
		sample_direction = vec3(map_coord.x, -1.0, map_coord.y);
	else if(LayerGS == 4)
		sample_direction = vec3(-map_coord.x, map_coord.y, 1.0);
	else if(LayerGS == 5)
		sample_direction = vec3(map_coord.xy, -1.0);

	float depth = texture(tex, normalize(sample_direction)).r;
	depth = linearize(depth);
	depth = spherical_depth(depth, phi_theta.x, phi_theta.y);

	FragColorLayer = depth;
}