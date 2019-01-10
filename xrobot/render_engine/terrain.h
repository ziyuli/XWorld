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
	kBoxBlur,
	kPerlin,
	kLighting,
	kTerrainDeferred,
	kTerrainForward,
	kTerrainShadow,
	kTerrainShaders
};

class Terrain {
public:
	Terrain(const int grid_size, const int terrain_size);
	~Terrain();

	void LoadTerrainTextures(const std::vector<std::string> albedo,
							 const std::vector<std::string> normal);

	void GenerateTextureID();

	void GenerateHeightMap(TerrainDatas_t* terrain_data);

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
	void GenerateAuxMaps(TerrainDatas_t* terrain_data);
	void GenerateHeight();
	void GenerateNormalMap();
	void RenderTerrain(Shader& shader,
					   const glm::vec3 camera,
					   const glm::mat4 view,
		        	   const glm::mat4 projection,
		        	   const glm::mat4 crop = glm::mat4(1));

public:
	std::vector<float> height_data_;
	std::vector<Shader> shaders_;
	GLuint quad_vao_, quad_vbo_;
	GLuint terrain_vao_;
	int grid_size_, terrain_size_, grid_high_res_;

	std::shared_ptr<RenderTarget> noise_;
	std::shared_ptr<RenderTarget> noise_high_res_;
	std::shared_ptr<RenderTarget> normal_;
	std::shared_ptr<RenderTarget> texture_id_high_res_;
	std::shared_ptr<RenderTarget> texture_id_blur_;

	GLuint height_scale_;
	GLuint texture_id_;
	GLuint terrain_textures_;
};


}}

#endif // RENDER_ENGINE_TERRAIN_H_
