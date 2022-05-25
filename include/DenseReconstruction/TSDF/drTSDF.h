#pragma once
#include "../../Common/cmTypeDefs.h"
#include "../../DenseReconstruction/Voxel/drVoxelStore.h"

namespace DenseReconstruction {
	class TruncatedSDF {
	public:
		void drIdealFirstTsdfEstimate(f64* depthMap, u32 imageWidth, u32 imageHeight, Common::Camera::MonocularCameraIntrinsic* cameraInt, VoxelStore* outStore, f64 truncationValue);
		void drIdealTsdfEstimate(f64* depthMap, u32 imageWidth, u32 imageHeight);
	};
}