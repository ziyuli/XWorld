#include "terrain_shape.h"

namespace xrobot {
namespace render_engine {

TerrainShape::TerrainShape(const int grid_size, 
				           const int terrain_size) : grid_size_(grid_size),
										   			 grid_high_res_(grid_size_ * 2),
										   			 terrain_size_(terrain_size),
										   			 quad_vbo_(0),
										   			 quad_vao_(0),
										   			 terrain_vao_(0),
										   			 shaders_(0),
										   			 texture_scale_(0),
										   			 textures_(0),
										   			 path_cache_("") {

	height_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	height_->append_rgb_float16_bilinear();
	height_->init();

	normal_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	normal_->append_rgb_float16_bilinear();
	normal_->init();

	blend_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	blend_->append_rgba_float16_bilinear();
	blend_->append_rgba_float16_bilinear();
	blend_->init();

	blend_blur_ = std::make_shared<RenderTarget>(grid_high_res_, grid_high_res_);
	blend_blur_->append_rgba_float16_bilinear();
	blend_blur_->append_rgba_float16_bilinear();
	blend_blur_->init();

	readback_ = std::make_shared<RenderTarget>(grid_size_, grid_size_);
	readback_->append_r_float();
	readback_->init();

	glGenTextures(1, &height_raw_low_);
	glGenTextures(2, layer_blend_raw_low_);

	InitShaders();
}

TerrainShape::~TerrainShape() {
	glDeleteVertexArrays(1, &terrain_vao_);
	glDeleteVertexArrays(1, &quad_vao_);
	glDeleteBuffers(1, &quad_vbo_);
	glDeleteTextures(1, &height_raw_low_);
	glDeleteTextures(2, layer_blend_raw_low_);
	glDeleteTextures(1, &textures_);
}

void TerrainShape::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kTerrainShaders);

	shaders_[kTerrainBlend] = Shader(pwd+"/shaders/quad.vs",
							         pwd+"/shaders/terrain_blur.fs");

	shaders_[kBlit] = Shader(pwd+"/shaders/quad.vs",
							 pwd+"/shaders/flat.fs");

	shaders_[kUpRes] = Shader(pwd+"/shaders/quad.vs",
							  pwd+"/shaders/terrain_up_res.fs");

	shaders_[kTerrainHeight] = Shader(pwd+"/shaders/quad.vs",
							          pwd+"/shaders/terrain_height.fs");

	shaders_[kTerrainNormal] = Shader(pwd+"/shaders/quad.vs",
							   	      pwd+"/shaders/terrain_normal.fs");

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

void TerrainShape::LoadTerrainTextures(TerrainLayers_t* layers) {

	const TerrainLayers_t terrain_layers = *layers;

	texture_scale_.clear();

	std::vector<std::string> paths;
	std::string path_key;
	for (int i = 0; i < terrain_layers.size(); ++i) {
		path_key += terrain_layers[i].path;
		paths.push_back(terrain_layers[i].path);
		texture_scale_.push_back(terrain_layers[i].scale);
	}

	if(path_key != path_cache_) {
		if(textures_ > 0)
			glDeleteTextures(1, &textures_);

		textures_ = LoadTextureArray(paths);
		path_cache_ = path_key;
	}
}

void TerrainShape::LoadTerrainParameters(const glm::vec2 ns,
							        const glm::vec3 clp,
							        const glm::vec2 seed) {
	height_clamp_ = clp;
	noise_scale_ = ns;
	seed_ = seed;
}

void TerrainShape::Reset() {}

void TerrainShape::LoadTerrainData(TerrainDatas_t* terrain_data) {
	height_data_.clear();
	height_data_.resize(grid_size_ * grid_size_);
	
	TerrainDataToTextures(terrain_data);
	GenerateHeightMap();
	GenerateNormalMap();
	GenerateBlendMap();
}

void TerrainShape::GenerateBlendMap() {
	auto upres = shaders_[kUpRes];
	auto blur = shaders_[kTerrainBlend];

	// up-res
	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, blend_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	upres.use();
	upres.setInt("tex0", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, layer_blend_raw_low_[0]);
	upres.setInt("tex1", 1);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, layer_blend_raw_low_[1]);

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// blur
	glBindFramebuffer(GL_FRAMEBUFFER, blend_blur_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	blur.use();
	blur.setInt("tex0", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, blend_->texture_id(0));
	blur.setInt("tex1", 1);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, blend_->texture_id(1));

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void TerrainShape::TerrainDataToTextures(TerrainDatas_t* terrain_data) {

	std::vector<glm::vec4> texture_blend_array_0;
	std::vector<glm::vec4> texture_blend_array_1;
	texture_blend_array_0.resize(grid_size_ * grid_size_);
	texture_blend_array_1.resize(grid_size_ * grid_size_);

	std::vector<float> height_array;
	height_array.resize(grid_size_ * grid_size_);

	const TerrainDatas_t terrain = *terrain_data;

	for (int i = 0; i < grid_size_ * grid_size_; ++i) {
		texture_blend_array_0[i] = terrain[i].texture_layer_masks[0];
		texture_blend_array_1[i] = terrain[i].texture_layer_masks[1];
		height_array[i] = terrain[i].height;
	}

	glBindTexture(GL_TEXTURE_2D, layer_blend_raw_low_[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, grid_size_, grid_size_,
			0, GL_RGBA, GL_FLOAT, &texture_blend_array_0[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glBindTexture(GL_TEXTURE_2D, layer_blend_raw_low_[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, grid_size_, grid_size_,
			0, GL_RGBA, GL_FLOAT, &texture_blend_array_1[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glBindTexture(GL_TEXTURE_2D, height_raw_low_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, grid_size_, grid_size_,
			0, GL_RED, GL_FLOAT, &height_array[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

void TerrainShape::GenerateHeightMap() {
	auto height = shaders_[kTerrainHeight];
	auto blit = shaders_[kBlit];

	// height map
	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, height_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	height.use();
	height.setVec2("seed", seed_);
	height.setVec3("height_clamp", height_clamp_);
	height.setFloat("perlin_high_freq", noise_scale_.y);
	height.setFloat("perlin_low_freq", noise_scale_.x);
	height.setInt("height_map", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, height_raw_low_);

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// blit
	glBindFramebuffer(GL_FRAMEBUFFER, readback_->id());
	glViewport(0, 0, grid_size_, grid_size_);
	glClear(GL_COLOR_BUFFER_BIT);

	blit.use();
	blit.setInt("tex", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, height_->texture_id(0));

	RenderQuad();
	glReadPixels(0, 0, grid_size_, grid_size_, GL_RED, GL_FLOAT, &height_data_[0]);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void TerrainShape::GenerateNormalMap() {
	auto normal = shaders_[kTerrainNormal];

	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, normal_->id());
	glViewport(0, 0, grid_high_res_, grid_high_res_);
	glClear(GL_COLOR_BUFFER_BIT);

	normal.use();
	normal.setFloat("scale", (float) terrain_size_ / grid_high_res_);
	normal.setInt("height_map", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, height_->texture_id(0));

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void TerrainShape::RenderTerrainDeferred(const glm::vec3 camera,
					        	    const glm::mat4 view,
		        	        	    const glm::mat4 projection) {
	auto terrain = shaders_[kTerrainDeferred];

	RenderTerrain(terrain, camera, view, projection);
}

void TerrainShape::RenderTerrainForward(const glm::vec3 camera,
					          const glm::mat4 view,
		        	          const glm::mat4 projection) {
	auto terrain = shaders_[kTerrainForward];

	RenderTerrain(terrain, camera, view, projection);
}

void TerrainShape::RenderTerrainPSSM(const glm::vec3 camera,
		        	   			const glm::mat4 crop) {
	auto terrain = shaders_[kTerrainShadow];

	RenderTerrain(terrain, camera, glm::mat4(1), glm::mat4(1), crop);
}

void TerrainShape::RenderTerrain(Shader& shader,
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
	shader.setInt("blend_map_0", 2);
	shader.setInt("blend_map_1", 3);
	shader.setInt("terrain_maps", 4);
	for (int i = 0; i < texture_scale_.size(); ++i) {
		shader.setFloat("terrain_scale[" + std::to_string(i) + "]",
						texture_scale_[i]);
	}

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, normal_->texture_id(0));
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, height_->texture_id(0));
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, blend_blur_->texture_id(0));
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, blend_blur_->texture_id(1));
	glActiveTexture(GL_TEXTURE4);
	glBindTexture(GL_TEXTURE_2D_ARRAY, textures_);

	glDrawArraysInstanced(GL_PATCHES, 0, 4, chunk_per_side * chunk_per_side);

	glBindVertexArray(0);
}

void TerrainShape::RenderQuad() {
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