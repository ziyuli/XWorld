#ifndef RENDER_ENGINE_TERRAIN_H_
#define RENDER_ENGINE_TERRAIN_H_

#include <string>
#include <vector>

#include "gl_context.h"
#include "shader.h"
#include "texture2d.h"
#include "common.h"
#include "render_target.h"
#include "render_world.h"
#include "render_utils.h"

namespace xrobot {
namespace render_engine {

enum TerrainShaders{
	kBlit,
	kUpRes,
	kTerrainHeight,
	kTerrainNormal,
	kTerrainBlend,
	kTerrainDeferred,
	kTerrainForward,
	kTerrainShadow,
	kTerrainShaders
};

class TerrainShape {
public:
	TerrainShape(const int grid_size, const int terrain_size);
	~TerrainShape();
	void Reset();

	void LoadTerrainTextures(TerrainLayers_t* layers);
	void LoadTerrainData(TerrainDatas_t* terrain_data);
	void LoadTerrainParameters(const glm::vec2 ns,
							   const glm::vec3 clp,
							   const glm::vec2 seed);

	void RenderTerrainForward(const glm::vec3 camera,
					          const glm::mat4 view,
		        	          const glm::mat4 projection);

	void RenderTerrainDeferred(const glm::vec3 camera,
					   		   const glm::mat4 view,
		        	   		   const glm::mat4 projection);

	void RenderTerrainPSSM(const glm::vec3 camera,
		        	   	   const glm::mat4 crop);

	std::vector<float> GetHeightMapData() const { return height_data_; };

private:
	void InitShaders();
	void RenderQuad();
	void TerrainDataToTextures(TerrainDatas_t* terrain_data);
	void GenerateHeightMap();
	void GenerateNormalMap();
	void GenerateBlendMap();
	void RenderTerrain(Shader& shader,
					   const glm::vec3 camera,
					   const glm::mat4 view,
		        	   const glm::mat4 projection,
		        	   const glm::mat4 crop = glm::mat4(1));

public:
	std::vector<float> texture_scale_;
	std::vector<float> height_data_;
	std::vector<Shader> shaders_;
	std::string path_cache_;
	GLuint quad_vao_, quad_vbo_;
	GLuint terrain_vao_;
	int grid_size_, terrain_size_, grid_high_res_;

	glm::vec3 height_clamp_;
	glm::vec2 noise_scale_;
	glm::vec2 seed_;

	GLuint height_raw_low_;
	GLuint layer_blend_raw_low_[2];
	GLuint textures_;

	std::shared_ptr<RenderTarget> readback_;
	std::shared_ptr<RenderTarget> height_;
	std::shared_ptr<RenderTarget> normal_;
	std::shared_ptr<RenderTarget> blend_blur_;
	std::shared_ptr<RenderTarget> blend_;
};


}}

#endif // RENDER_ENGINE_TERRAIN_H_
