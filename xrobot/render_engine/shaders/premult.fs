#version 330 core

out vec4 FragColor;

in mediump vec2 TexCoords;

const highp float ep = 0.005f;

// Disk
uniform lowp float show_disk = 1.0f;
uniform highp float aspect_ratio = 1.0f;
uniform highp float disk_radius = 0.01f;
uniform highp float alpha = 1.0f;
uniform vec3 disk_color = vec3(0.9f, 0.0f, 0.0f);
uniform vec2 disk_center = vec2(0.5f, 0.5f);

// Inventory
uniform float start_u;
uniform float end_u;

uniform mediump float zNear = 0.02;
uniform mediump float zFar = 70.0;
uniform lowp float deferred = 0;
uniform sampler2D tex;
uniform sampler2D dep;

float linearize(float depth) 
{
	return (2 * zNear) / (zFar + zNear - depth * (zFar - zNear));
}

void main()
{
	FragColor.rgb = texture(tex, TexCoords).rgb;

	if(deferred > 0)
	{
		FragColor.a = linearize(texture(dep, TexCoords).a);
		if(texture(tex, TexCoords).a < 1) {
			FragColor.rgb = vec3(0.5f);
			FragColor.a = 1.0f;
		}
	}
	else
	{
		FragColor.a = linearize(texture(dep, TexCoords).r);
	}

	// Draw Disk
	vec2 uv_offset = (TexCoords - disk_center) * vec2(aspect_ratio, 1);
	float dist = sqrt(dot(uv_offset, uv_offset));
	float t = smoothstep(disk_radius + ep, disk_radius - ep, dist) * alpha;
	FragColor.rgb = mix(FragColor.rgb, disk_color, t); 

	// Draw Inventory
	if(TexCoords.x > start_u && TexCoords.x < end_u && TexCoords.y < 0.1) 
	{
		FragColor.rgb = vec3(0.3);
	}

}