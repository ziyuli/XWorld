#include "navigation_on_terrain.h"

namespace xrobot
{
	Task_Terrain::Task_Terrain(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<Map> map) : iterations_(0),
						            scene_(map),
						            agent_(),
						            renderer_(renderer),
						            ctx_(renderer->GetContext()),
						            main_camera_(nullptr),
						            cam_pitch_(0) {
		house_ = std::make_shared<Suncg>(map->world_);
		terrain_ = std::make_shared<Terrain>(map->world_);
		renderer_->UpdateAmbientColor(glm::vec3(1.0f));
		renderer_->UpdateExposure(1.0f);
	}

	Task_Terrain::~Task_Terrain() {}

	TaskStages Task_Terrain::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_Terrain::Start, this);
		stages["NavTarget"] = std::bind(&Task_Terrain::NavTarget, this);
		return stages;
	}

	std::string Task_Terrain::Start() {

		std::vector<glm::vec3> path_control_points;
		path_control_points.push_back(glm::vec3( 5, 0.2, 6));
		path_control_points.push_back(glm::vec3(10, 0.2, 6));
		path_control_points.push_back(glm::vec3(15, 0.2, 6));
		path_control_points.push_back(glm::vec3(20, 0.2, 6));
		path_control_points.push_back(glm::vec3(20, 0.2, 11));
		path_control_points.push_back(glm::vec3(15, 0.2, 15));

        iterations_ = 0;
        terrain_->Reset();
        house_->Reset();
		scene_->ResetMap();
		terrain_->AddTerrainLayer(texture_gravel, 5.0f);
		terrain_->AddTerrainLayer(texture_road, 20.0f);
		terrain_->AddTerrainLayer(texture_grass, 20.0f);
		terrain_->AddTerrainLayer(texture_stone, 20.0f);
		terrain_->GenerateTerrain(30, 128, 2.0);
		terrain_->GeneratePuddle(glm::vec2(10, 10), 10, 0, 2);
		terrain_->GeneratePuddle(glm::vec2(10, 15), 10, 0, 2);
		terrain_->GeneratePath(path_control_points, 32, 4, 1);
		terrain_->LoadTerrain();

	    // Spawn Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky,
	        glm::vec3(15,0.5,15),
	        glm::vec3(-1,0,0), 1.57,
	        glm::vec3(0.8f),
	        "agent",
	        false
	    );

	   	house_->SetRemoveAll( kRemoveDoor );
	   	house_->LoadCategoryCSV(suncg_meta.c_str());
	    house_->LoadJSON(suncg_house.c_str(), 
	    				suncg_dir.c_str(),
	    				true,
	    				glm::vec3(10,0,10));
    	

	   	if(auto agent_sptr = agent_.lock()) 
	   	{
	   		agent_sptr->ignore_baking(true);
	    	agent_sptr->DisableSleeping();

		   	// Create a Camera and Attach to the Agent
		    main_camera_ = scene_->world_->add_camera(vec3(0, 0, 0),
		    		vec3(0.3,0.8,0.0), (float) 4 / 3);
		    scene_->world_->attach_camera(main_camera_, agent_sptr.get());
		}

	    scene_->world_->BulletStep();

	    return "NavTarget";
	}

	std::string Task_Terrain::NavTarget() {

		pos_0_ = 0.0f;
        pos_1_ = 0.0f;
        pos_2_ = 0.0f;
        pos_3_ = 0.0f;

		if(ctx_->GetKeyPressUp()) {
            pos_0_ += 3.5f;
            pos_1_ += 3.5f;
            pos_2_ += 3.5f;
            pos_3_ += 3.5f;
		}

        if(ctx_->GetKeyPressDown()) {
            pos_0_ -= 3.5f;
            pos_1_ -= 3.5f;
            pos_2_ -= 3.5f;
            pos_3_ -= 3.5f;
        }

        if(ctx_->GetKeyPressLeft()) {
            pos_0_ -= 3.5f;
            pos_1_ += 3.5f;
            pos_2_ -= 3.5f;
            pos_3_ += 3.5f;
        }

        if(ctx_->GetKeyPressRight()) {
            pos_0_ += 3.5f;
            pos_1_ -= 3.5f;
            pos_2_ += 3.5f;
            pos_3_ -= 3.5f;
        }

        std::shared_ptr<Joint> j;
        if(auto agent_sptr = agent_.lock()) 
	   	{
	        j = agent_sptr->joints_[2];
	        j->SetJointMotorControlVelocity(pos_0_, 0.1f, 50000.0f);
	        j = agent_sptr->joints_[3];
	        j->SetJointMotorControlVelocity(pos_1_, 0.1f, 50000.0f);
	        j = agent_sptr->joints_[4];
	        j->SetJointMotorControlVelocity(pos_2_, 0.1f, 50000.0f);
	        j = agent_sptr->joints_[5];
	        j->SetJointMotorControlVelocity(pos_3_, 0.1f, 50000.0f);
	    }

    	// Step Simulation and Renderer
        scene_->world_->BulletStep();   
        renderer_->StepRender(scene_->world_.get());
        return "NavTarget";
	}

}