#pragma once
#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelDefs.h"

namespace DenseReconstruction {
	class VoxelStore {
	public:
		i32 ox = 0, oy = 0, oz = 0;
		i32 width = 0, height = 0, depth = 0;
	public:
		virtual void drInitialize(i32 width, i32 height, i32 depth, i32 ox, i32 oy, i32 oz, f64 scale){}
		virtual void drSetVoxel(i32 x, i32 y, i32 z, Voxel* value){}
		virtual void drGetVoxel(i32 x, i32 y, i32 z, Voxel** value){}
	};
}