#include "test_terrain.h"

namespace xrobot
{
	Task_Terrain::Task_Terrain(
		std::shared_ptr<render_engine::Render> renderer,
		std::shared_ptr<MapTerrain> map) : iterations_(0),
								           scene_(map),
								           agent_(),
								           renderer_(renderer),
								           ctx_(renderer->GetContext()),
								           main_camera_(nullptr),
								           cam_pitch_(0) {}

	Task_Terrain::~Task_Terrain() {}

	TaskStages Task_Terrain::GetStages() {
		TaskStages stages;
		stages["idle"] = std::bind(&Task_Terrain::Start, this);
		stages["NavTarget"] = std::bind(&Task_Terrain::NavTarget, this);
		return stages;
	}

	std::string Task_Terrain::Start() {

		std::vector<glm::vec3> curve;
		curve.push_back(glm::vec3(10, -0.1, 10));
		curve.push_back(glm::vec3(10, -0.1, 20));
		curve.push_back(glm::vec3(20, -0.1, 20));
		curve.push_back(glm::vec3(10, -0.1, 20));

		// Reset
        iterations_ = 0;
		scene_->ResetMap();
		scene_->GenerateTerrain(30, 128);
		scene_->GeneratePuddle(glm::vec2(10,10), 7, 0.1, 0);
		scene_->GeneratePath(curve, 3, 3, 1);
		scene_->LoadTerrain();

		renderer_->UpdateAmbientColor(glm::vec3(1.0f));
		renderer_->UpdateExposure(2.0f);

	    // Spawn Agent
	   	agent_ = scene_->world_->LoadRobot(
	        husky,
	        glm::vec3(2,0.5,2),
	        glm::vec3(-1,0,0), 1.57,
	        glm::vec3(1.0f, 1.0f, 1.0f),
	        "agent",
	        false
	    );

	   	scene_->world_->LoadRobot(
        	"../data/house/house.urdf",
        	glm::vec3(10.0, -0.2, 10.0),
        	glm::vec3(1,0,0), 0,
        	glm::vec3(0.005f),
        	"house",
        	true
    	);

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
            pos_0_ += 2.5f;
            pos_1_ += 2.5f;
            pos_2_ += 2.5f;
            pos_3_ += 2.5f;
		}

        if(ctx_->GetKeyPressDown()) {
            pos_0_ -= 2.5f;
            pos_1_ -= 2.5f;
            pos_2_ -= 2.5f;
            pos_3_ -= 2.5f;
        }

        if(ctx_->GetKeyPressLeft()) {
            pos_0_ -= 2.5f;
            pos_1_ += 2.5f;
            pos_2_ -= 2.5f;
            pos_3_ += 2.5f;
        }

        if(ctx_->GetKeyPressRight()) {
            pos_0_ += 2.5f;
            pos_1_ -= 2.5f;
            pos_2_ += 2.5f;
            pos_3_ -= 2.5f;
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