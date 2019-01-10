#include "map_terrain.h"

namespace xrobot
{
	MapTerrain::~MapTerrain() {}

	MapTerrain::MapTerrain() : Map(),
							   grid_size_(0),
							   terrain_size_(0) {

		world_ = std::make_shared<World>();
		world_->BulletInit(-9.81f, 0.01f);

		std::random_device rd;
		mt_ = std::mt19937(rd());
	}

	void MapTerrain::ResetMap() {
		if(world_->reset_count_ % 1000) {
			world_->CleanEverything2();
		} else {
			world_->CleanEverything();
		}

		printf("[World] Reset %d\n", world_->reset_count_);
	}

	void MapTerrain::GenerateTerrain(const int terrain_size, 
									 const int grid_size) {
		
		terrain_size_ = terrain_size;
		grid_size_ = grid_size;
		scale_ = glm::vec2(grid_size_ / (float) terrain_size_);
		terrain_.resize(grid_size_ * grid_size_);

		for (int i = 0; i < grid_size_ * grid_size_; ++i) {
			terrain_[i] = {0, 1, false};
		}
	}

	void MapTerrain::LoadTerrain() {
		world_->GenerateTerrain(grid_size_, terrain_size_, &terrain_);
		world_->set_world_size(0, 0, terrain_size_, terrain_size_);
	}

	void MapTerrain::GeneratePath(const std::vector<glm::vec3> control_points,
			const int num_points, const float& scale, const unsigned int id) {

		const int perc = 2;
		std::vector<glm::vec3> curve_points;

		for(int i = 0; i <= perc; ++i) {
			float u = float(i) / float(perc);
			glm::vec3 point(0);

			for (int m = 0; m <= num_points; ++m) {
				float bm = bernstein(m, num_points, u);
				point.x += bm * control_points[m].x;
				point.y += bm * control_points[m].y;
				point.z += bm * control_points[m].z;
				curve_points.push_back(point);
			}
		}

		for (int i = 0; i < curve_points.size(); ++i) {
			glm::vec3 point = curve_points[i];
			GenerateCircle(glm::vec2(point.x, point.z), scale, point.y, id);
		}
	}

	void MapTerrain::GeneratePuddle(const glm::vec2& position, const float& scale, 
			const float height, const unsigned int id) {

		for (int i = 0; i < random(3, 6); ++i) {
			float x = random(-1.0f, 1.0f) * scale * 0.5f;
			float z = random(-1.0f, 1.0f) * scale * 0.5f;
			GenerateCircle(position + glm::vec2(x, z), 2.0f * scale, height, id);
		}
	}


	void MapTerrain::GenerateCircle(const glm::vec2& position, const float& radius, 
			const float height, const unsigned int id) {
		
		for (int i = 0; i < grid_size_ * grid_size_; ++i) {
			float x = i % grid_size_;
			float z = i / grid_size_;

			glm::vec2 point(x, z);
			if(glm::length(point - position * scale_) < radius) {
				terrain_[i] = {id, height, true};
			}
		}
	}

	void MapTerrain::GenerateBox(const glm::vec2& aabb_min, const glm::vec2& aabb_max, 
			const float height, const unsigned int id) {

		for (int i = 0; i < grid_size_ * grid_size_; ++i) {
			float x = i % grid_size_;
			float z = i / grid_size_;

			glm::vec2 point(x, z);
			if(point.x > aabb_min.x * scale_.x && point.y > aabb_min.y * scale_.y &&
			   point.x < aabb_max.x * scale_.x && point.y < aabb_max.y * scale_.y) {
				terrain_[i] = {id, height, true};
			}
		}
	}
}
