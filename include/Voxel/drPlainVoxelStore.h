#pragma once

#include "drVoxelStore.h"

#define allocate_mem(tp,size) (new tp[size])
#define free_mem(obj) (delete[] obj)

#define get_pixel3(im,x,y,z,w,h,d) ((im)[(y)*(w)*(d)+(x)*(d)+(z)])

namespace DenseReconstruction {
	class PlainVoxelStore: public VoxelStore {
	public:
		Voxel* store = nullptr;
		
	public:
		void drInitialize(int width, int height, int depth, int ox, int oy, int oz, double scale) override;
		void drSetVoxel(int x, int y, int z, Voxel* value) override;
		void drGetVoxel(int x, int y, int z, Voxel** value) override;
		~PlainVoxelStore();
	};
}