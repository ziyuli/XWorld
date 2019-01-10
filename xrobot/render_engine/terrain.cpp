#include "terrain.h"

namespace xrobot {
namespace render_engine {

Terrain::Terrain(const int grid_size, 
				 const int terrain_size) : grid_size_(grid_size),
										   grid_high_res_(grid_size_ * 2),
										   terrain_size_(terrain_size),
										   quad_vbo_(0),
										   quad_vao_(0),
										   terrain_vao_(0),
										   shaders_(0) {

	noise_ = std::make_shared<RenderTarget>(grid_size_, grid_size_);
	noise_->append(GL_LINEAR, GL_LINEAR, GL_RGB16F, GL_RGB, 
			GL_FLOAT, GL_CLAMP_TO_BORDER);
	noise_->init();

	noise_high_res_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	noise_high_res_->append(GL_LINEAR, GL_LINEAR, GL_RGB16F, GL_RGB, 
			GL_FLOAT, GL_CLAMP_TO_BORDER);
	noise_high_res_->init();

	normal_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	normal_->append_rgba_float16_bilinear();
	normal_->init();

	texture_id_high_res_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	texture_id_high_res_->append_rgb_float_bilinear();
	texture_id_high_res_->init();

	texture_id_blur_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	texture_id_blur_->append_rgb_float_bilinear();
	texture_id_blur_->init();

	glGenTextures(1, &height_scale_);
	glGenTextures(1, &texture_id_);

	InitShaders();

	std::vector<std::string> textures_path(0);
	LoadTerrainTextures(textures_path,textures_path);
}

Terrain::~Terrain() {
	glDeleteVertexArrays(1, &terrain_vao_);
	glDeleteVertexArrays(1, &quad_vao_);
	glDeleteBuffers(1, &quad_vbo_);
	glDeleteTextures(1, &height_scale_);
	glDeleteTextures(1, &texture_id_);
	glDeleteTextures(1, &terrain_textures_);
}

void Terrain::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kTerrainShaders);

	shaders_[kBoxBlur] = Shader(pwd+"/shaders/quad.vs",
							    pwd+"/shaders/box.fs");

	shaders_[kBlit] = Shader(pwd+"/shaders/quad.vs",
							 pwd+"/shaders/flat.fs");

	shaders_[kPerlin] = Shader(pwd+"/shaders/quad.vs",
							   pwd+"/shaders/perlin.fs");

	shaders_[kLighting] = Shader(pwd+"/shaders/quad.vs",
							   	 pwd+"/shaders/terrain_shading.fs");

	shaders_[kTerrainDeferred] = Shader(pwd+"/shaders/terrain.vs",
							            pwd+"/shaders/terrain_deferred.fs",
							            pwd+"/shaders/terrain.tcs",
							            pwd+"/shaders/terrain.tes");

	shaders_[kTerrainForward] = Shader(pwd+"/shaders/terrain.vs",
							           pwd+"/shaders/terrain_forward.fs",
							           pwd+"/shaders/terrain.tcs",
							           pwd+"/shaders/terrain.tes");

	shaders_[kTerrainShadow] = Shader(pwd+"/shaders/terrain.vs",
							          pwd+"/shaders/pssm.fs",
							          pwd+"/shaders/terrain.tcs",
							          pwd+"/shaders/terrain_pssm.tes");
}

void Terrain::LoadTerrainTextures(const std::vector<std::string> albedo,
							 	  const std::vector<std::string> normal) {
	
	// hardcode...
	std::vector<std::string> textures_path;
	textures_path.push_back("/home/ziyuli/XWorld/xrobot/data/gravel.jpg");
	textures_path.push_back("/home/ziyuli/XWorld/xrobot/data/street.jpg");

	terrain_textures_ = LoadTextureArray(textures_path);
}

void Terrain::GenerateTextureID() {
	auto blur = shaders_[kBoxBlur];
	auto blit = shaders_[kBlit];

	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, texture_id_high_res_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	blit.use();
	blit.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texture_id_);

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glBindFramebuffer(GL_FRAMEBUFFER, texture_id_blur_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	blur.use();
	blur.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texture_id_high_res_->texture_id(0));

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Terrain::GenerateHeightMap(TerrainDatas_t* terrain_data) {
	height_data_.clear();
	height_data_.resize(grid_size_ * grid_size_);
	
	GenerateAuxMaps(terrain_data);
	GenerateHeight();
	GenerateNormalMap();
	GenerateTextureID();
}

void Terrain::GenerateAuxMaps(TerrainDatas_t* terrain_data) {

	std::vector<float> texture_id_array;
	texture_id_array.resize(grid_size_ * grid_size_);

	std::vector<float> height_scale_array;
	height_scale_array.resize(grid_size_ * grid_size_);

	const TerrainDatas_t terrain = *terrain_data;

	for (int i = 0; i < grid_size_ * grid_size_; ++i) {
		texture_id_array[i] = terrain[i].texture_layer_id;
		height_scale_array[i] = terrain[i].height;
	}

	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glTexImage2D(GL_TEXTURE_2D, 0, 
			GL_R8, grid_size_, grid_size_, 0, GL_RED, GL_FLOAT, &texture_id_array[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glBindTexture(GL_TEXTURE_2D, height_scale_);
	glTexImage2D(GL_TEXTURE_2D, 0, 
			GL_R16F, grid_size_, grid_size_, 0, GL_RED, GL_FLOAT, &height_scale_array[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

void Terrain::GenerateHeight() {
	auto perlin = shaders_[kPerlin];
	auto blit = shaders_[kBlit];

	// high-res
	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, noise_high_res_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	perlin.use();
	perlin.setInt("height_scale_map", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, height_scale_);

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// blit
	glBindFramebuffer(GL_FRAMEBUFFER, noise_->id());
	glViewport(0, 0, grid_size_, grid_size_);
	glClear(GL_COLOR_BUFFER_BIT);

	blit.use();
	blit.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, noise_high_res_->texture_id(0));

	RenderQuad();
	glReadPixels(0, 0, grid_size_, grid_size_, GL_RED, GL_FLOAT, &height_data_[0]);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Terrain::GenerateNormalMap() {
	auto lighting = shaders_[kLighting];

	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, normal_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	lighting.use();
	lighting.setFloat("scale", (float) terrain_size_ / grid_high_res_);
	lighting.setInt("height_map", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, noise_high_res_->texture_id(0));

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Terrain::RenderTerrainDeferred(const glm::vec3 camera,
					        	    const glm::mat4 view,
		        	        	    const glm::mat4 projection) {
	auto terrain = shaders_[kTerrainDeferred];

	RenderTerrain(terrain, camera, view, projection);
}

void Terrain::RenderTerrainForward(const glm::vec3 camera,
					          const glm::mat4 view,
		        	          const glm::mat4 projection) {
	auto terrain = shaders_[kTerrainForward];

	// glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	RenderTerrain(terrain, camera, view, projection);

	// glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Terrain::RenderTerrainPSSM(const glm::vec3 camera,
		        	   			const glm::mat4 crop) {
	auto terrain = shaders_[kTerrainShadow];

	RenderTerrain(terrain, camera, glm::mat4(1), glm::mat4(1), crop);
}

void Terrain::RenderTerrain(Shader& shader,
							const glm::vec3 camera,
							const glm::mat4 view,
		        			const glm::mat4 projection,
		        			const glm::mat4 crop) {

	if(terrain_vao_ == 0) {
		glGenVertexArrays(1, &terrain_vao_);
	}

	const int chunk_size = 5;
	const int chunk_per_side = terrain_size_ / chunk_size;

	glPatchParameteri(GL_PATCH_VERTICES, 4);

	glBindVertexArray(terrain_vao_);

	shader.use();
	shader.setMat4("projection", projection);
	shader.setMat4("view", view);
	shader.setMat4("crop", crop);
	shader.setVec3("camera_pos", camera);
	shader.setInt("terrain_size", terrain_size_);
	shader.setInt("span_size", terrain_size_);
	shader.setInt("chunk_size", chunk_size);
	shader.setInt("height_map_size", grid_size_);
	shader.setInt("texture_map_size", grid_size_);
	shader.setInt("normal_map", 0);
	shader.setInt("height_map", 1);
	shader.setInt("texture_id_map", 2);
	shader.setInt("terrain_maps", 3);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, normal_->texture_id(0));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, noise_high_res_->texture_id(0));
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, texture_id_blur_->texture_id(0));
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D_ARRAY, terrain_textures_);

	glDrawArraysInstanced(GL_PATCHES, 0, 4, chunk_per_side * chunk_per_side);

	glBindVertexArray(0);
}

void Terrain::RenderQuad() {
	if (quad_vao_ == 0) {
		constexpr float quadVertices[] = {
			-1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
			-1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
			1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
			1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		};
			
		glGenVertexArrays(1, &quad_vao_);
		glGenBuffers(1, &quad_vbo_);
		glBindVertexArray(quad_vao_);
		glBindBuffer(GL_ARRAY_BUFFER, quad_vbo_);
		glBufferData(GL_ARRAY_BUFFER,
					 sizeof(quadVertices),
					 &quadVertices,
					 GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(
				0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1,
							  2,
							  GL_FLOAT,
							  GL_FALSE,
							  5 * sizeof(float),
							  (void*)(3 * sizeof(float)));
	}
	glBindVertexArray(quad_vao_);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindVertexArray(0);
}



}
}