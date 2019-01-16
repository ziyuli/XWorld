#version 330 core

layout (location = 0) out vec4 blend_layer_0;
layout (location = 1) out vec4 blend_layer_1;

in vec2 TexCoords;

uniform sampler2D tex0;
uniform sampler2D tex1;

const ivec3 off = ivec3(-1,0,1);

vec4 upres(sampler2D tex)
{
	vec4 h00 = texture(tex, TexCoords);
    vec4 h01 = textureOffset(tex, TexCoords, off.xy);
    vec4 h21 = textureOffset(tex, TexCoords, off.zy);
    vec4 h10 = textureOffset(tex, TexCoords, off.yx);
    vec4 h12 = textureOffset(tex, TexCoords, off.yz);
    return mix(h00, mix(mix(h01, h10, 0.5), mix(h21, h12, 0.5), 0.5), 0.5);
}

void main() 
{
	blend_layer_0 = upres(tex0);
	blend_layer_1 = upres(tex1);
}