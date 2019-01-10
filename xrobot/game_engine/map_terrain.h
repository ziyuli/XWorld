#ifndef MAP_TERRAIN_H_
#define MAP_TERRAIN_H_

#include "map.h"
#include <random>

using namespace glm;

namespace xrobot
{
	class MapTerrain : public Map
	{
	public:
		MapTerrain();
		~MapTerrain();

		void GeneratePath(const std::vector<glm::vec3> control_points,
						  const int num_points, const float& scale,
						  const unsigned int id);

		void GeneratePuddle(const glm::vec2& position, const float& scale, 
				const float height, const unsigned int id);
		void GenerateCircle(const glm::vec2& position, const float& radius, 
				const float height, const unsigned int id);
		void GenerateBox(const glm::vec2& aabb_min, const glm::vec2& aabb_max, 
				const float height, const unsigned int id);

		void LoadTerrain();
		void GenerateTerrain(const int terrain_size, const int grid_size);
		void ResetMap();

	private:

		int grid_size_;
		int terrain_size_;
		glm::vec2 scale_;
		std::mt19937 mt_;
		std::vector<TerrainData> terrain_;

		inline float random(const float a, const float b) {
			std::uniform_real_distribution<float> rnd(a, b);
			return rnd(mt_);
		}

		inline int factorial(const int n) {
			int r = 1;
			for(int i = n; i > 0; i--) 
				r *= i;
			return r;
		}

		inline float bernstein(const int i, const int n, const float t) {
			float r = (float) factorial(n) / (float) (factorial(i) * factorial(n - i));
			r *= pow(t, i);
			r *= pow(1 - t, n - i);
			return r;
		}


	};
}

#endif // MAP_TERRAIN_H_

