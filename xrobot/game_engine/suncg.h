#ifndef SUNCG_H_
#define SUNCG_H_

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

namespace xrobot
{
	enum SUNCGModelRemoveTypes {
		kRemoveDoor = 1,
		kRemoveStairs = 2,
	};

	struct Properity {
		float mass;
		bool concave;
	};

	class Suncg
	{
	public:
		Suncg(std::shared_ptr<World> world);
		~Suncg();

		int GetJsonArrayEntry(Json::Value *&result, Json::Value *array,
				unsigned int k, int expected_type = -1);
		int GetJsonObjectMember(Json::Value *&result, Json::Value *object,
				const char *str, int expected_type = 0);
		void LoadJSON(const char * houseFile, const char * input_data_directory,
				const bool concave = false, const vec3 offset = vec3(0, 0, 0));
		void LoadCategoryCSV(const char * metadataFile);
		void SetRemoveAll(const unsigned int remove);
		void SetRemoveRandomly(const bool remove) { remove_randomly_ = remove; }
		void Reset();
		void AddPhysicalProperties(const std::string& label, const Properity& prop);

		std::unordered_map<int, std::string> map_bullet_label_; // Not Useful?????
		std::unordered_map<std::string, std::string> all_labels_;
		std::unordered_map<std::string, Properity> map_labels_properity;

	private:
		float GetRandom(const float low, const float high);

		std::shared_ptr<World> world_;

		bool remove_all_doors_;
		bool remove_all_stairs_;
		bool remove_randomly_;

		std::random_device rand_device_;
		std::mt19937 mt_;
	};
}

#endif // SUNCG_H_
