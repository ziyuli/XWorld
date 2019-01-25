#version 330 core
#extension GL_EXT_texture_array : enable

out vec4 FragColor;

in vec2 TexCoords;

uniform int layer = 0;
uniform sampler2DArray tex;

void main()
{
    FragColor = vec4(texture(tex, vec3(TexCoords, float(layer))).rgb, 1);
}

