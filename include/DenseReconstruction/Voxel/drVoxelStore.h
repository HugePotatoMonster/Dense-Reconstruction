#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelDefs.h"

namespace DenseReconstruction {
	class VoxelStore {
	public:
		i32 ox = 0, oy = 0, oz = 0;
		u32 width = 0, height = 0, depth = 0;
	public:
		virtual void drInitialize(u32 width, u32 height, u32 depth, i32 ox, i32 oy, i32 oz, f64 scale) = 0;
		virtual void drSetVoxel(u32 x, u32 y, u32 z, Voxel* value) = 0;
		virtual void drGetVoxel(u32 x, u32 y, u32 z, Voxel** value) = 0;
	};
}