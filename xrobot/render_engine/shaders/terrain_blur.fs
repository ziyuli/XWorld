#version 330 core

layout (location = 0) out vec4 blend_layer_0;
layout (location = 1) out vec4 blend_layer_1;

in mediump vec2 TexCoords;

uniform sampler2D tex0;
uniform sampler2D tex1;

vec4 box_blur(sampler2D tex)
{
    vec2 texelSize = 1.0 / vec2(textureSize(tex, 0));
    vec4 result = vec4(0);

    for (int x = -3; x < 3; ++x) 
    {
        for (int y = -3; y < 3; ++y) 
        {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            vec4 v = texture(tex, TexCoords + offset).xyzw;
            result += v;
        }
    }

    return result / (6.0 * 6.0);
}

void main() 
{
    blend_layer_0 = box_blur(tex0);
    blend_layer_1 = box_blur(tex1);
}  