#pragma once
#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelStore.h"

namespace DenseReconstruction {
	class PlainVoxelStore: public VoxelStore {
	public:
		Voxel* store = nullptr;
		
	public:
		void drInitialize(i32 width, i32 height, i32 depth, i32 ox, i32 oy, i32 oz, f64 scale) override;
		void drSetVoxel(i32 x, i32 y, i32 z, Voxel* value) override;
		void drGetVoxel(i32 x, i32 y, i32 z, Voxel** value) override;
		~PlainVoxelStore();
	};
}