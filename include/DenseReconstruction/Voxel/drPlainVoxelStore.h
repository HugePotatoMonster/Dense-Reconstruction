#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelStore.h"

namespace DenseReconstruction {
	class PlainVoxelStore:public VoxelStore {
	private:
		Voxel* store = nullptr;
		i32 ox = 0, oy = 0, oz = 0;
	public:
		void drInitialize(u32 width, u32 height, u32 depth, i32 ox, i32 oy, i32 oz, f64 scale) override;
		void drSetVoxel(u32 x, u32 y, u32 z, Voxel* value) override;
		void drGetVoxel(u32 x, u32 y, u32 z, Voxel* value) override;
		~PlainVoxelStore();
	};
}