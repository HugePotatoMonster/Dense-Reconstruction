#pragma once

#include "../Common/cmTypeDefs.h"
#include "drVoxelDefs.h"

namespace DenseReconstruction {
	class VoxelStore {
	public:
		int ox = 0, oy = 0, oz = 0;
		int width = 0, height = 0, depth = 0;
	public:
		virtual void drInitialize(int width, int height, int depth, int ox, int oy, int oz, double scale){}
		virtual void drSetVoxel(int x, int y, int z, Voxel* value){}
		virtual void drGetVoxel(int x, int y, int z, Voxel** value){}
	};
}