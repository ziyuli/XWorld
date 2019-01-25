#ifndef RENDER_ENGINE_CAPTURE_H_
#define RENDER_ENGINE_CAPTURE_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "gl_context.h"
#include "common.h"
#include "shader.h"
#include "csm.h"
#include "texture2d.h"
#include "texture3d.h"
#include "render_target.h"
#include "render_world.h"

namespace xrobot {
namespace render_engine {

enum CaptureShaders {
    kCapture,
    kSphericalProjection,
    kCubemap,
    kCaptureShaders
};

class Capture {
public:
	Capture(const int resolution);
	~Capture();

	void SphericalProjection();
	void Stitch(GLuint& rgb, Image<float>& lidar_image);
	void RenderCubemap(RenderWorld* world, 
					   Camera* camera);

	GLuint GetRawCubeMap() const { return raw_capture_cubemap_; }
	glm::vec3 Front() const { return front_; }
	glm::vec3 Up() const { return up_; }
	void SetFront(const glm::vec3 front) { front_ = front; }
	void SetUp(const glm::vec3 up) { up_ = up; }

private:
	void InitPBOs();
	void InitShaders();
	void InitCapture();
	void InitSphericalProjection();
	void Draw(RenderWorld* world, const Shader& shader);
	void RenderQuad();

	int resolution_;
	GLuint raw_capture_fb_;
	GLuint raw_capture_cubemap_;

	GLuint spherical_projection_fb_;
	GLuint spherical_projection_texarray_;

	GLuint quad_vao_, quad_vbo_;
	GLuint lidar_pbo_;

	glm::vec3 front_, up_;

	std::shared_ptr<RenderTarget> capture_;

	std::vector<Shader> shaders_;
};

}
}

#endif // RENDER_ENGINE_CAPTURE_H_
