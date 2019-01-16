#version 330 core

precision highp float;

out float FragColor;

noperspective in vec2 TexCoords;

uniform sampler2D height_map;
uniform vec3  height_clamp     = vec3(-1, 1, 0);
uniform float perlin_high_freq = 0.02;
uniform float perlin_low_freq  = 0.01;
uniform vec2  seed             = vec2(0);

const ivec3 off = ivec3(-1,0,1);

vec4 permute(vec4 x){ return mod(((x*34.0)+1.0)*x, 289.0); }

vec2 fade(vec2 t) { return t*t*t*(t*(t*6.0-15.0)+10.0); }

float rand(vec2 co){
	return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 123.456);
}

float cnoise(vec2 P) {
	vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
	vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
	Pi = mod(Pi, 289.0);
	vec4 ix = Pi.xzxz;
	vec4 iy = Pi.yyww;
	vec4 fx = Pf.xzxz;
	vec4 fy = Pf.yyww;
	vec4 i = permute(permute(ix) + iy);
	vec4 gx = 2.0 * fract(i * 0.0243902439) - 1.0;
	vec4 gy = abs(gx) - 0.5;
	vec4 tx = floor(gx + 0.5);
	gx = gx - tx;
	vec2 g00 = vec2(gx.x,gy.x);
	vec2 g10 = vec2(gx.y,gy.y);
	vec2 g01 = vec2(gx.z,gy.z);
	vec2 g11 = vec2(gx.w,gy.w);
	vec4 norm = 1.79284291400159 - 0.85373472095314 * 
			vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11));
	g00 *= norm.x;
	g01 *= norm.y;
	g10 *= norm.z;
	g11 *= norm.w;
	float n00 = dot(g00, vec2(fx.x, fy.x));
	float n10 = dot(g10, vec2(fx.y, fy.y));
	float n01 = dot(g01, vec2(fx.z, fy.z));
	float n11 = dot(g11, vec2(fx.w, fy.w));
	vec2 fade_xy = fade(Pf.xy);
	vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
	return 2.3 * mix(n_x.x, n_x.y, fade_xy.y);
}

float perlin(float scale) {
	float noise = cnoise((TexCoords + rand(seed)) * scale);
	return clamp(noise, height_clamp.x, height_clamp.y);
}

float height(vec2 P) {
	float h00 = texture(height_map, P).x;
	float h01 = textureOffset(height_map, P, off.xy).x;
    float h21 = textureOffset(height_map, P, off.zy).x;
    float h10 = textureOffset(height_map, P, off.yx).x;
    float h12 = textureOffset(height_map, P, off.yz).x;
    return mix(h00, mix(mix(h01, h10, 0.5), mix(h21, h12, 0.5), 0.5), 0.5);
}

void main() {
	float perlin_low  = perlin(4.0f) * perlin_low_freq;
	float perlin_high = perlin(12.0f) * perlin_high_freq;
	float height_raw  = height(TexCoords);
	if(height_clamp.z < 1)
		FragColor = perlin_low + perlin_high + height_raw;
	else
		FragColor = (perlin_low + perlin_high) * height_raw;
}