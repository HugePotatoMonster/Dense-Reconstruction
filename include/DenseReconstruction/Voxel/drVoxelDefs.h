#pragma once
#include "../../Common/cmTypeDefs.h"

#define VOXEL_BLOCK_W 8
#define VOXEL_BLOCK_H 8 
#define VOXEL_BLOCK_D 8

namespace DenseReconstruction {
	struct Voxel {
		f64 tsdf;
		u8 color[3];
		u32 weights;
	};
	struct VoxelBlock {
		Voxel child[VOXEL_BLOCK_D * VOXEL_BLOCK_H * VOXEL_BLOCK_W];
	};
}