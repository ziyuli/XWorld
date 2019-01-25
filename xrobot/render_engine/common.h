#ifndef RENDER_ENGINE_COMMON_H_
#define RENDER_ENGINE_COMMON_H_

#include <chrono>
#include <ctime>
#include <ratio>
#include <stdio.h>
#include <string>
#include <vector>

#include "gl_context.h"
#include "shader.h"
#include "csm.h"
#include "texture2d.h"
#include "texture3d.h"
#include "render_target.h"

namespace xrobot {
namespace render_engine {

enum kRenderFeatures {
    kNone = 0,
    kShading = 1,
    kShadow = 2,
    kAO = 4,
    kSR = 8,
    kVCT = 16,
    kAA = 32,
    kVisualization = 64,
    kDepthCapture = 128
};

constexpr int kDefaultConf = kShading | kVisualization;
constexpr int kVeryLow = kNone;
constexpr int kLow = kShading;
constexpr int kNormal = kShading | kAO | kAA;
constexpr int kMed = kShading | kAO | kAA | kShadow;
constexpr int kHigh = kShading | kAO | kAA | kShadow | kSR;
constexpr int kExtreme = kShading | kAO | kAA | kShadow | kSR | kVCT;
constexpr int kVeryLowVisualize = kVeryLow | kVisualization;
constexpr int kLowVisualize = kLow | kVisualization;
constexpr int kNormalVisualize = kNormal | kVisualization;
constexpr int kMedVisualize = kMed | kVisualization;
constexpr int kHighVisualize = kHigh | kVisualization;
constexpr int kExtremeVisualize = kExtreme | kVisualization;
constexpr int kLidarCaptureRes = 128;

const int profiles[6] = {kVeryLow,
                         kLow,
                         kNormal,
                         kMed,
                         kHigh,
                         kExtreme};

struct Lighting {
    float exposure = 0.5f;
    glm::vec3 bg = glm::vec3(0.5f);
};

struct DirectionalLight {
    glm::vec3 direction = glm::vec3(1, 2, 1);
    glm::vec3 diffuse = glm::vec3(0.8f, 0.8f, 0.8f);
    glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);
    glm::vec3 ambient = glm::vec3(0.1f, 0.1f, 0.1f);
};

struct PSSM {
    CSMUniforms csm_uniforms;
    CSM csm;
    int shadow_map_size = 2048;
    int cascade_count = 4;
    float pssm_lamda = 0.65f;
    float near_offset = 80.0f;
    float shadow_bias_clamp = 0.0005f;
    float shadow_bias_scale = 0.0005f;
    bool first_run = true;
};

template <typename T>
struct Image {
    Image() : data(0) {}
    std::vector<T> data;
};

static std::string get_pwd(const std::string& file) {
    size_t p = file.find_last_of("/");
    return file.substr(0, p);
}

}
}

#endif // RENDER_ENGINE_COMMON_H_
