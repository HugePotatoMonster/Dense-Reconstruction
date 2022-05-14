#include "../../../include/SemiGlobalMatching/Helper/smCostHelper.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smCensusTransformCostCalculator.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smFourPathCostAggregator.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smEightPathCostAggregator.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smNullPathCostAggregator.h"
#include "../../../include/SemiGlobalMatching/CostOptimizer/smCostOptimizer.h"

namespace SemiGlobalMatching {
	void CostHelper::calculateCost(u8* imageLeft, u8* imageRight, u32 imageWidth, u32 imageHeight,i32 minDisparity, u32 disparityRange, u32* leftDisparityMap) {
		SemiGlobalMatching::CostCalculator* costEstimator = new SemiGlobalMatching::CensusTransformCostCalculator();
		SemiGlobalMatching::CostAggregator* costAggregator = new SemiGlobalMatching::FourPathCostAggregator();
		SemiGlobalMatching::CostOptimizer* costOptimizer = new SemiGlobalMatching::CostOptimizer();

		u8* costMatrix = allocate_mem(u8, imageWidth * imageHeight * disparityRange);
		costEstimator->smCostCalculate(imageLeft, imageRight, imageWidth, imageHeight,minDisparity, disparityRange, costMatrix);

		u32* refinedCostMatrix = allocate_mem(u32, imageWidth * imageHeight * disparityRange);
		set_zero(refinedCostMatrix, sizeof(u32) * imageWidth * imageHeight * disparityRange);
		costAggregator->smCostAggregate(imageLeft, costMatrix, imageWidth, imageHeight, minDisparity, disparityRange, refinedCostMatrix);

		f64* disparityMapLeft = allocate_mem(f64, imageWidth * imageHeight);
		f64* disparityMapLeftS = allocate_mem(f64, imageWidth * imageHeight);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrix, disparityMapLeft, disparityMapLeftS, imageWidth, imageHeight, minDisparity, disparityRange);
		costOptimizer->smDisparityMapDiscretization(disparityMapLeft, leftDisparityMap, imageWidth, imageHeight, disparityRange, disparityRange);
	
		free_mem(disparityMapLeft);
		free_mem(refinedCostMatrix);
		free_mem(costMatrix); 
	}

	void CostHelper::calculateCostInternalF(u8* imageLeft, u8* imageRight, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, f64* leftDisparityMap, f64* leftDisparitymapS, u32* occuList, u32* occuLen, u32* misList, u32* misLen) {
		SemiGlobalMatching::CostCalculator* costEstimator = new SemiGlobalMatching::CensusTransformCostCalculator();
		SemiGlobalMatching::CostAggregator* costAggregator = new SemiGlobalMatching::EightPathCostAggregator();
		SemiGlobalMatching::CostOptimizer* costOptimizer = new SemiGlobalMatching::CostOptimizer();

		u8* costMatrix = allocate_mem(u8, imageWidth * imageHeight * disparityRange);
		costEstimator->smCostCalculate(imageLeft, imageRight, imageWidth, imageHeight, minDisparity, disparityRange, costMatrix);

		u32* refinedCostMatrix = allocate_mem(u32, imageWidth * imageHeight * disparityRange);
		set_zero(refinedCostMatrix, sizeof(u32) * imageWidth * imageHeight * disparityRange);
		costAggregator->smCostAggregate(imageLeft, costMatrix, imageWidth, imageHeight, minDisparity, disparityRange, refinedCostMatrix);

		u32* refinedCostMatrixRight = allocate_mem(u32, imageWidth * imageHeight * disparityRange);
		costEstimator->smGetAnotherCost(refinedCostMatrix, imageWidth, imageHeight, minDisparity, disparityRange, refinedCostMatrixRight);

		f64* disparityMapLeft = allocate_mem(f64, imageWidth * imageHeight);
		f64* disparityMapRight = allocate_mem(f64, imageWidth * imageHeight); 
		f64* disparityMapRightS = allocate_mem(f64, imageWidth * imageHeight);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrix, disparityMapLeft, leftDisparitymapS, imageWidth, imageHeight, minDisparity, disparityRange);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrixRight, disparityMapRight, disparityMapRightS, imageWidth, imageHeight, minDisparity, disparityRange);

		costOptimizer->smInternalConsistencyCheckF(disparityMapLeft, disparityMapRight, leftDisparityMap, imageWidth, imageHeight, occuList, occuLen, misList, misLen, 1.0, SGM_INVALID_DISPARITY_F);

		free_mem(disparityMapRight);
		free_mem(disparityMapRightS);
		free_mem(disparityMapLeft);
		free_mem(refinedCostMatrixRight);
		free_mem(refinedCostMatrix);
		free_mem(costMatrix);
	}
}