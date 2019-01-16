#include "terrain.h"

namespace xrobot
{
	Terrain::~Terrain() {}

	Terrain::Terrain(std::shared_ptr<World> world) : world_(world),
							   						 grid_size_(0),
							   						 terrain_size_(0) {
		std::random_device rd;
		mt_ = std::mt19937(rd());
	}

	void Terrain::Reset() {
		terrain_data_.clear();
		terrain_layers_.clear();
	}

	void Terrain::GenerateTerrain(const int terrain_size, 
							      const int grid_size,
							      const float height) {
		
		terrain_size_ = terrain_size;
		grid_size_ = grid_size;
		scale_ = glm::vec2(grid_size_ / (float) terrain_size_);
		terrain_data_.resize(grid_size_ * grid_size_);

		for (int i = 0; i < grid_size_ * grid_size_; ++i) {
			terrain_data_[i] = TerrainData();
			terrain_data_[i].height = height;
			terrain_data_[i].SetBlend(0, 1);
		}
	}

	void Terrain::LoadTerrain(const glm::vec3 clamp, const glm::vec2 noise) {
		assert(terrain_layers_.size());

		glm::vec2 seed = glm::vec2(random(-100, 100), random(-100, 100));

		auto world_sptr = wptr_to_sptr(world_);
		world_sptr->GenerateTerrain(grid_size_, terrain_size_, noise, clamp, seed,
				&terrain_data_, &terrain_layers_);
		world_sptr->set_world_size(0, 0, terrain_size_, terrain_size_);
	}

	void Terrain::AddTerrainLayer(const std::string& path, const float scale) {
		assert(terrain_layers_.size() < 8);
		terrain_layers_.push_back({path, scale});
	}

	void Terrain::GeneratePath(const std::vector<glm::vec3> control_points,
			const int prec, const float& scale, const unsigned int id) {

		int num_points = control_points.size() - 1;
		std::vector<glm::vec3> curve_points;

		for(int i = 0; i <= prec; ++i) {
			float u = float(i) / float(prec);
			glm::vec3 point(0);

			for (int m = 0; m <= num_points; ++m) {
				float bm = bernstein(m, num_points, u);
				point.x += bm * control_points[m].x;
				point.y += bm * control_points[m].y;
				point.z += bm * control_points[m].z;
			}
			curve_points.push_back(point);
		}

		for (int i = 0; i < curve_points.size(); ++i) {
			glm::vec3 point = curve_points[i];
			GenerateCircle(glm::vec2(point.x, point.z), scale, point.y, id);
		}
	}

	void Terrain::GeneratePuddle(const glm::vec2& position, const float& scale, 
			const float height, const unsigned int id) {

		for (int i = 0; i < random(5, 10); ++i) {
			float x = random(-1.0f, 1.0f) * scale * 0.5f;
			float z = random(-1.0f, 1.0f) * scale * 0.5f;
			GenerateCircle(position + glm::vec2(x, z), 2.0f * scale, height, id);
		}
	}


	void Terrain::GenerateCircle(const glm::vec2& position, const float& radius, 
			const float height, const unsigned int id) {
		
		for (int i = 0; i < grid_size_ * grid_size_; ++i) {
			float x = i % grid_size_;
			float z = i / grid_size_;

			glm::vec2 point(x, z);
			if(glm::length(point - position * scale_) < radius) {
				terrain_data_[i].height = height;
				terrain_data_[i].occupy = true;
				terrain_data_[i].SetBlendOverride(id, 1.0f);
			}
		}
	}

	void Terrain::GenerateBox(const glm::vec2& aabb_min, const glm::vec2& aabb_max, 
			const float height, const unsigned int id) {

		for (int i = 0; i < grid_size_ * grid_size_; ++i) {
			float x = i % grid_size_;
			float z = i / grid_size_;

			glm::vec2 point(x, z);
			if(point.x > aabb_min.x * scale_.x && point.x < aabb_max.x * scale_.x &&
			   point.y > aabb_min.y * scale_.y && point.y < aabb_max.y * scale_.y) {
				terrain_data_[i].height = height;
				terrain_data_[i].occupy = true;
				terrain_data_[i].SetBlendOverride(id, 1.0f);
			}
		}
	}
}
