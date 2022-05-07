#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelDefs.h"

namespace DenseReconstruction {
	class VoxelStore {
	public:
		virtual void drInitialize(u32 width, u32 height, u32 depth, i32 ox, i32 oy, i32 oz, f64 scale) = 0;
		virtual void drSetVoxel(u32 x, u32 y, u32 z, Voxel* value) = 0;
		virtual void drGetVoxel(u32 x, u32 y, u32 z, Voxel* value) = 0;
	};
}