#ifndef RENDER_ENGINE_FBO_H
#define RENDER_ENGINE_FBO_H

#include <iostream>
#include <string>
#include <vector>

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

class FBO {
public:
	GLuint width;
    GLuint height;
	GLuint frameBuffer;

	bool needDepthBuffer;
    bool needColor1Buffer;

    FBO(GLuint w,
        GLuint h,
        bool depthBuffer = false,
        bool color1Buffer = false,
		GLenum magFilter = GL_NEAREST,
        GLenum minFilter = GL_NEAREST,
		GLint internalFormat = GL_RGBA8,
        GLint format = GL_UNSIGNED_BYTE,
		GLint wrap = GL_CLAMP_TO_BORDER);

	~FBO();

	void ActivateAsTexture(const int shaderProgram,
                           const std::string& glSamplerName,
                           const int textureUnit = 0);

	void ActivateDepthAsTexture(const int shaderProgram,
                                const std::string& glSamplerName,
                                const int textureUnit = 0);

	void ActivateColor1AsTexture(const int shaderProgram,
                             const std::string& glSamplerName,
                             const int textureUnit = 0);
    private:
        GLuint textureColor0Buffer;
        GLuint textureColor1Buffer;
        GLuint textureDepthBuffer;
        GLuint rbo;
        GLuint attachement;
};

} } // xrobot::render_engine

#endif
