#ifndef TASK_EXAMPLE_TERRAIN_H_
#define TASK_EXAMPLE_TERRAIN_H_

#include <vector>
#include <functional>
#include <unordered_map>

#include "render_engine/render.h"

#include "game_engine/lidar.h"
#include "game_engine/suncg.h"
#include "game_engine/terrain.h"
#include "game_engine/navigation.h"
#include "game_engine/state_machine.h"
#include "game_engine/task.h"

#include "assets.h"
#include "utils.h"

namespace xrobot
{
	typedef std::unordered_map<std::string,
							   std::function<std::string()>> TaskStages;

	class Task_Terrain : public TaskInterface {
	public:

		Task_Terrain(std::shared_ptr<render_engine::Render> renderer,
				     std::shared_ptr<Map> map);
		~Task_Terrain();

		std::string Start();
		std::string NavTarget();
		TaskStages GetStages();

		int iterations_;
		float cam_pitch_;

		std::weak_ptr<RobotBase> agent_;
		std::shared_ptr<Suncg> house_;
		std::shared_ptr<Terrain> terrain_;
		std::shared_ptr<Map> scene_;
		std::shared_ptr<render_engine::Render> renderer_;

		render_engine::GLContext * ctx_;
		render_engine::Camera * main_camera_;

		float pos_0_ = 0.0f;
	    float pos_1_ = 0.0f;
	    float pos_2_ = 0.0f;
	    float pos_3_ = 0.0f;
	};
}

#endif // TASK_EXAMPLE_TERRAIN_H_