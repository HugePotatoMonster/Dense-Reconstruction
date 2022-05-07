#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelStore.h"

namespace DenseReconstruction {
	class PlainVoxelStore: public VoxelStore {
	public:
		Voxel* store = nullptr;
		
	public:
		void drInitialize(u32 width, u32 height, u32 depth, i32 ox, i32 oy, i32 oz, f64 scale) override;
		void drSetVoxel(u32 x, u32 y, u32 z, Voxel* value) override;
		void drGetVoxel(u32 x, u32 y, u32 z, Voxel** value) override;
		~PlainVoxelStore();
	};
}