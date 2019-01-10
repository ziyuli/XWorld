#include "render_engine/render.h"
#include "utils.h"

#include <fstream>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <unistd.h>
#include <ios>
#include <iomanip>
#include <memory>

#include "game_engine/task.h"
#include "game_engine/state_machine.h"
#include "test_terrain.h"

using namespace xrobot;

constexpr int w = 640;
constexpr int h = 480;

int main(int argc, char **argv)
{
    assert(argc < 3);

    render_engine::Profile profile = render_engine::kVeryLowQualityVisualize;
    if(argc == 2 && atoi(argv[1]) < 5) 
        profile = render_engine::profiles[atoi(argv[1])];

    std::shared_ptr<MapTerrain> scene = std::make_shared<MapTerrain>();
    std::shared_ptr<render_engine::Render> renderer = 
            std::make_shared<render_engine::Render> (w, h, profile, false);
    
    TaskGroup group("TaskGroup_Terrain");
    Task_Terrain task_nav(renderer, scene);
    group.AddTask("Terrain", task_nav);

    while(!renderer->GetContext()->GetWindowShouldClose())
        group.RunStage();

    return 0;
}