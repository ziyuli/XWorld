#include "fbo.h"

namespace xrobot {
namespace render_engine {

FBO::FBO(GLuint w,
         GLuint h,
         bool depthBuffer,
         bool color1Buffer,
         GLenum magFilter,
         GLenum minFilter,
         GLint internalFormat,
         GLint format,
         GLint wrap) {
	this->width = w;
	this->height = h;
	this->needDepthBuffer = depthBuffer;
    this->needColor1Buffer = color1Buffer;

	GLint previousFrameBuffer;
	glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFrameBuffer);

	glGenFramebuffers(1, &frameBuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

	glGenTextures(1, &textureColor0Buffer);
	glBindTexture(GL_TEXTURE_2D, textureColor0Buffer);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);

	glTexImage2D(GL_TEXTURE_2D,
                 0,
                 internalFormat,
                 w,
                 h,
                 0,
                 GL_RGBA,
                 format,
                 NULL);
	glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           textureColor0Buffer,
                           0);

	if (depthBuffer == false) {
		glGenRenderbuffers(1, &rbo);
		glBindRenderbuffer(GL_RENDERBUFFER, rbo);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
		glFramebufferRenderbuffer(
                GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);

		unsigned int attachments[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, attachments);
	} else {
		glGenTextures(1, &textureDepthBuffer);
		glBindTexture(GL_TEXTURE_2D, textureDepthBuffer);
		glTexImage2D(GL_TEXTURE_2D,
                     0,
                     GL_DEPTH_COMPONENT24,
                     w,
                     h,
                     0,
                     GL_DEPTH_COMPONENT,
                     GL_FLOAT,
                     0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_ALPHA);
		glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D,
                               textureDepthBuffer,
                               0);
	}
    
    if (color1Buffer) {
        glGenTextures(1, &textureColor1Buffer);
        glBindTexture(GL_TEXTURE_2D, textureColor1Buffer);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     internalFormat,
                     w,
                     h,
                     0,
                     GL_RGBA,
                     format,
                     NULL);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT1,
                               GL_TEXTURE_2D,
                               textureColor1Buffer,
                               0);
        
        unsigned int attach[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        glDrawBuffers(2, attach);
    }

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cout << "FBO failed to initialize correctly." << std::endl;
	}

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

FBO::~FBO() {
	glDeleteTextures(1, &textureColor0Buffer);
	glDeleteFramebuffers(1, &frameBuffer);
	if (needDepthBuffer == false) {
        glDeleteRenderbuffers(1, &rbo);
    } else {
        glDeleteTextures(1, &textureDepthBuffer);
    }
    if (needColor1Buffer) {
        glDeleteTextures(1, &textureColor1Buffer);
    }
}

void FBO::ActivateColor1AsTexture(const int shaderProgram,
                                  const std::string& glSamplerName,
                                  const int textureUnit) {
    if (!this->needColor1Buffer) { return; }
    glActiveTexture(GL_TEXTURE0 + textureUnit);
    glBindTexture(GL_TEXTURE_2D, textureColor1Buffer);
    GLuint loc = glGetUniformLocation(shaderProgram, glSamplerName.c_str());
    glUniform1i(loc, textureUnit);
}


void FBO::ActivateAsTexture(const int shaderProgram,
                            const std::string& glSamplerName,
                            const int textureUnit) {
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_2D, textureColor0Buffer);
    GLuint loc = glGetUniformLocation(shaderProgram, glSamplerName.c_str());
	glUniform1i(loc, textureUnit);
}

void FBO::ActivateDepthAsTexture(const int shaderProgram,
                                 const std::string& glSamplerName,
                                 const int textureUnit) {
	if (!this->needDepthBuffer) { return; }
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_2D, textureDepthBuffer);
    GLuint loc = glGetUniformLocation(shaderProgram, glSamplerName.c_str());
	glUniform1i(loc, textureUnit);
}

} } // xrobot::render_engine
