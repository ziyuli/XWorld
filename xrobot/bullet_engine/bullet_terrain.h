#ifndef BULLET_TERRAIN_H_
#define BULLET_TERRAIN_H_

#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletTerrain {
public:
	BulletTerrain() : id_(-1) {}

	virtual ~BulletTerrain() {}

	void load_terrain(const ClientHandle client, 
			void* data, const int grid_size, const int size,
			const float height_min, const float height_max);

	void remove_from_bullet(const ClientHandle client);

public:
	int id_;
};

}} // namespace xrobot::bullet_engine

#endif
