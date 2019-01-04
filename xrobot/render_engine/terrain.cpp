#include "terrain.h"

namespace xrobot {
namespace render_engine {

Terrain::Terrain(const int size) : size_(size),
								   quad_vbo_(0),
								   shaders_(0) {

	noise_ = std::make_shared<RenderTarget>(size, size);
	noise_->append_r_float();
	noise_->init();

	lightmap_ = std::make_shared<RenderTarget>(size, size);
	lightmap_->append_rgb_uint8();
	lightmap_->init();

	InitShaders();

	GeneratePerlinNoise();

	GenerateLightMap();

	GenerateTerrain();
}

Terrain::~Terrain() {
	glDeleteVertexArrays(1, &quad_vao_);
	glDeleteBuffers(1, &quad_vbo_);
}

void Terrain::InitShaders() {
	std::string pwd = get_pwd(__FILE__);
	shaders_.resize((int) kTerrainShaders);

	shaders_[kPerlin] = Shader(pwd+"/shaders/quad.vs",
							   pwd+"/shaders/perlin.fs");

	shaders_[kLighting] = Shader(pwd+"/shaders/quad.vs",
							   	 pwd+"/shaders/terrain_shading.fs");

	shaders_[kTerrain] = Shader(pwd+"/shaders/terrain.vs",
							    pwd+"/shaders/terrain.fs",
							    pwd+"/shaders/terrain.tcs",
							    pwd+"/shaders/terrain.tes");
}


void Terrain::GeneratePerlinNoise() {
	auto perlin = shaders_[kPerlin];

	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, noise_->id());
	glViewport(0, 0, size_, size_);
	glClear(GL_COLOR_BUFFER_BIT);

	perlin.use();

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Terrain::GenerateLightMap() {
	auto lighting = shaders_[kLighting];

	glDisable(GL_DEPTH_TEST);
	glBindFramebuffer(GL_FRAMEBUFFER, lightmap_->id());
	glViewport(0, 0, size_, size_);
	glClear(GL_COLOR_BUFFER_BIT);

	lighting.use();
	lighting.setInt("height_map", 0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, noise_->texture_id(0));

	RenderQuad();
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Terrain::RenderTerrain(const glm::mat4 view,
		        			const glm::mat4 projection) {
	auto terrain = shaders_[kTerrain];

	
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