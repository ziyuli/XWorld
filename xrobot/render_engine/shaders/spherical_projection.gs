#version 330 core

layout (triangles) in;
layout (triangle_strip, max_vertices=18) out;

in vec2 TexCoords[3];

flat out int LayerGS;

out vec2 TexCoordsGS;

void main() {
    for(int face = 0; face < 6; ++face) {
        for(int i = 0; i < 3; ++i) {
            gl_Layer = face;
            LayerGS = face;
            TexCoordsGS = TexCoords[i];
            gl_Position = gl_in[i].gl_Position;
            EmitVertex();
        }    
        EndPrimitive();
    }
} 