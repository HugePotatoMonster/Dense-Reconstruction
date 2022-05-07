#include "../../Common/cmTypeDefs.h"

namespace DenseReconstruction {
	class VoxelStore {
	public:
		virtual void drInitialize(u32 width, u32 height, u32 depth) = 0;

	};
}