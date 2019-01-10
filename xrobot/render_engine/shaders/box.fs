#version 330 core
out float FragColor;

in vec2 TexCoords;

uniform sampler2D tex;

void main() 
{
    vec2 texelSize = 1.0 / vec2(textureSize(tex, 0));
    float result = 0.0;

    for (int x = -3; x < 3; ++x) 
    {
        for (int y = -3; y < 3; ++y) 
        {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            
            float v = texture(tex, TexCoords + offset).r;
            result += v;
        }
    }
    FragColor = result / (6.0 * 6.0);
}  