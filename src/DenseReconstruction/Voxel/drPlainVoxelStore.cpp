#include "../../../include/DenseReconstruction/Voxel/drPlainVoxelStore.h"

namespace DenseReconstruction {
	void PlainVoxelStore::drInitialize(i32 width, i32 height, i32 depth, i32 ox, i32 oy, i32 oz, f64 scale) {
		this->ox = ox;
		this->oy = oy;
		this->oz = oz;
		this->width = width;
		this->height = height;
		this->depth = depth;
		this->store = allocate_mem(Voxel, width * height * depth);
	}
	void PlainVoxelStore::drSetVoxel(i32 x, i32 y, i32 z, Voxel* value) {
		get_pixel3(store, x - this->ox, y - this->oy, z - this->oz, width, height, depth).tsdf = value->tsdf;
		get_pixel3(store, x - this->ox, y - this->oy, z - this->oz, width, height, depth).color[0] = value->color[0];
		get_pixel3(store, x - this->ox, y - this->oy, z - this->oz, width, height, depth).color[1] = value->color[1];
		get_pixel3(store, x - this->ox, y - this->oy, z - this->oz, width, height, depth).color[2] = value->color[2];
		get_pixel3(store, x - this->ox, y - this->oy, z - this->oz, width, height, depth).weights = value->weights;
	}
	void PlainVoxelStore::drGetVoxel(i32 x, i32 y, i32 z, Voxel** value) {
		*value = &(get_pixel3(store, x - this->ox, y - this->oy, z - this->oz, width, height, depth));
	}
	PlainVoxelStore::~PlainVoxelStore() {
		free_mem(store);
	}
}