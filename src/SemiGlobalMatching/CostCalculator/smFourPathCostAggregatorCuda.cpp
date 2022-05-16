#include "../../../include/Parallel/CUDA/ccuDeclarations.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smFourPathCostAggregatorCuda.h"

namespace SemiGlobalMatching {
	void FourPathCostAggregatorCuda::smCostAggregate(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, u32* refinedMatrix) {
		pr_assert(cu_enabled);
		if (cu_enabled) {
			Parallel::CUDA::SemiGlobalMatching::CostAggregator::cusmParallelCostAggregationFourPathCaller(
				imageData, costMatrix, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrix, 0, 32, nullptr);
		}
	}
}