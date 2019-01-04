#ifndef RENDER_ENGINE_TERRAIN_H_
#define RENDER_ENGINE_TERRAIN_H_

#include <string>
#include <vector>

#include "gl_context.h"
#include "shader.h"
#include "texture2d.h"
#include "common.h"
#include "render_target.h"

namespace xrobot {
namespace render_engine {

enum TerrainShaders{
	kPerlin,
	kLighting,
	kTerrain,
	kTerrainShaders
};

class Terrain {
public:
	Terrain(const int size);
	~Terrain();

	void GeneratePerlinNoise();
	void GenerateLightMap();

	void RenderTerrain(const glm::mat4 view,
		        	   const glm::mat4 projection);

	void InitShaders();
	void RenderQuad();

// private:
	std::vector<Shader> shaders_;
	GLuint quad_vao_, quad_vbo_;
	int size_;

	std::shared_ptr<RenderTarget> noise_;
	std::shared_ptr<RenderTarget> lightmap_;
};


}}

#endif // RENDER_ENGINE_TERRAIN_H_
