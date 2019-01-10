#include "bullet_terrain.h"

namespace xrobot {
namespace bullet_engine {

void BulletTerrain::load_terrain(const ClientHandle client, 
		void* data, const int grid_size, const int size,
		const float height_min, const float height_max) {

	CommandHandle cmd_handle = b3LoadTerrainCommandInit(client,
            data, grid_size, size, 1.0, height_min, height_max, 1);
    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_TERRAIN_LOADING_COMPLETED) {
        fprintf(stderr, "Cannot load Terrain");
    }

    id_ = b3GetStatusBodyIndex(status_handle);
}

void BulletTerrain::remove_from_bullet(const ClientHandle client) {
    b3SubmitClientCommandAndWaitStatus(
            client, b3InitRemoveBodyCommand(client, id_));
}

}}