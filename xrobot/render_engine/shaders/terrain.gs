#version 410 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in TES_OUT {
    vec2 tc;
	vec3 world;
	vec3 eye;
} gs_in[];

out GS_OUT {
	vec2 tc;
	vec3 world;
	vec3 eye;
	vec3 norm;
} gs_out;

void main()
{
	vec3 p0 = gs_in[0].world;
	vec3 p1 = gs_in[1].world;
	vec3 p2 = gs_in[2].world;

	vec3 p01 = p0 - p1;
	vec3 p02 = p0 - p2;

	vec3 norm = normalize(cross(p01, p02));

	gl_Position = gl_in[0].gl_Position;
	gs_out.tc = gs_in[0].tc;
	gs_out.world = gs_in[0].world;
	gs_out.eye = gs_in[0].eye;
	gs_out.norm = norm;
	EmitVertex();

	gl_Position = gl_in[1].gl_Position;
	gs_out.tc = gs_in[1].tc;
	gs_out.world = gs_in[1].world;
	gs_out.eye = gs_in[1].eye;
	gs_out.norm = norm;
	EmitVertex();

	gl_Position = gl_in[2].gl_Position;
	gs_out.tc = gs_in[2].tc;
	gs_out.world = gs_in[2].world;
	gs_out.eye = gs_in[2].eye;
	gs_out.norm = norm;
	EmitVertex();


    EndPrimitive();
}