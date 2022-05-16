#pragma once

#define VOXEL_BLOCK_W 8
#define VOXEL_BLOCK_H 8 
#define VOXEL_BLOCK_D 8

namespace DenseReconstruction {
	struct Voxel {
		double tsdf;
		unsigned char color[3];
		unsigned int weights;
	};
	struct VoxelBlock {
		Voxel child[VOXEL_BLOCK_D * VOXEL_BLOCK_H * VOXEL_BLOCK_W];
	};
}