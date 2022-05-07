#pragma once
#include "../cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelStore.h"
#include <string>

namespace Common {
	namespace Util {
		class VisualizationExt {
		public:
			void cmuExportVoxelToObj(std::string fileName,DenseReconstruction::VoxelStore* store);
			void cmuTsdfBinarization(DenseReconstruction::VoxelStore* inStore, DenseReconstruction::VoxelStore* outStore);
		};
	}
}