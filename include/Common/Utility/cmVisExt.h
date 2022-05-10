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
			void cmuVoxelMarchingCubes(DenseReconstruction::VoxelStore* inStore, Common::Mesh::SimpleMesh* outMesh);
			void cmuExportMeshToObj(std::string fileName, Common::Mesh::SimpleMesh* mesh);
		};
	}
}