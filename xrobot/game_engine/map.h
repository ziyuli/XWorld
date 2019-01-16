#ifndef MAP_H_
#define MAP_H_

#include <iostream>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <memory>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/matrix_decompose.hpp"

#include "world.h"
#include "utils.h"
#include "vendor/json.h"

using namespace glm;

namespace xrobot {

class Map {
public:
    Map() : world_(nullptr) { GenerateMap(); }
    
    virtual ~Map() {}

    virtual void ResetMap() {
        if(world_) {
            if(world_->reset_count_ % 1000)
                world_->CleanEverything2();
            else
                world_->CleanEverything();
        }
    }

    void GenerateMap() { 
        if(!world_) {
            world_ = std::make_shared<World>();
            world_->BulletInit(-9.81f, 0.01f);
        }
    }

    std::shared_ptr<World> world_;
};

}

#endif // MAP_H_
