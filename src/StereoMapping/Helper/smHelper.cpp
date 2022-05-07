#include "../../../include/StereoMapping/Helper/smCostHelper.h"
#include "../../../include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"
#include "../../../include/StereoMapping/CostCalculator/smFourPathCostAggregator.h"
#include "../../../include/StereoMapping/CostOptimizer/smCostOptimizer.h"

namespace StereoMapping {
	void CostHelper::calculateCost(u8* imageLeft, u8* imageRight, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* leftDisparityMap) {
		StereoMapping::CostCalculator* costEstimator = new StereoMapping::CensusTransformCostCalculator();
		StereoMapping::CostAggregator* costAggregator = new StereoMapping::FourPathCostAggregator();
		StereoMapping::CostOptimizer* costOptimizer = new StereoMapping::CostOptimizer();

		u8* costMatrix = allocate_mem(u8, imageWidth * imageHeight * disparityRange);
		costEstimator->smCostCalculate(imageLeft, imageRight, imageWidth, imageHeight, disparityRange, costMatrix);

		u32* refinedCostMatrix = allocate_mem(u32, imageWidth * imageHeight * disparityRange);
		set_zero(refinedCostMatrix, sizeof(u32) * imageWidth * imageHeight * disparityRange);
		costAggregator->smCostAggregate(imageLeft, costMatrix, imageWidth, imageHeight, disparityRange, refinedCostMatrix);

		f64* disparityMapLeft = allocate_mem(f64, imageWidth * imageHeight);
		f64* disparityMapLeftS = allocate_mem(f64, imageWidth * imageHeight);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrix, disparityMapLeft, disparityMapLeftS, imageWidth, imageHeight, disparityRange);
		costOptimizer->smDisparityMapDiscretization(disparityMapLeft, leftDisparityMap, imageWidth, imageHeight, disparityRange, disparityRange);
	
		free_mem(disparityMapLeft);
		free_mem(refinedCostMatrix);
		free_mem(costMatrix); 
	}

	void CostHelper::calculateCostInternalF(u8* imageLeft, u8* imageRight, u32 imageWidth, u32 imageHeight, u32 disparityRange, f64* leftDisparityMap, f64* leftDisparitymapS, u32* occuList, u32* occuLen, u32* misList, u32* misLen) {
		StereoMapping::CostCalculator* costEstimator = new StereoMapping::CensusTransformCostCalculator();
		StereoMapping::CostAggregator* costAggregator = new StereoMapping::FourPathCostAggregator();
		StereoMapping::CostOptimizer* costOptimizer = new StereoMapping::CostOptimizer();

		u8* costMatrix = allocate_mem(u8, imageWidth * imageHeight * disparityRange);
		costEstimator->smCostCalculate(imageLeft, imageRight, imageWidth, imageHeight, disparityRange, costMatrix);

		u32* refinedCostMatrix = allocate_mem(u32, imageWidth * imageHeight * disparityRange);
		set_zero(refinedCostMatrix, sizeof(u32) * imageWidth * imageHeight * disparityRange);
		costAggregator->smCostAggregate(imageLeft, costMatrix, imageWidth, imageHeight, disparityRange, refinedCostMatrix);

		u32* refinedCostMatrixRight = allocate_mem(u32, imageWidth * imageHeight * disparityRange);
		costEstimator->smGetAnotherCost(refinedCostMatrix, imageWidth, imageHeight, disparityRange, refinedCostMatrixRight);

		f64* disparityMapLeft = allocate_mem(f64, imageWidth * imageHeight);
		f64* disparityMapRight = allocate_mem(f64, imageWidth * imageHeight); 
		f64* disparityMapRightS = allocate_mem(f64, imageWidth * imageHeight);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrix, disparityMapLeft, leftDisparitymapS, imageWidth, imageHeight, disparityRange);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrixRight, disparityMapRight, disparityMapRightS, imageWidth, imageHeight, disparityRange);

		costOptimizer->smInternalConsistencyCheckF(disparityMapLeft, disparityMapRight, leftDisparityMap, imageWidth, imageHeight, occuList, occuLen, misList, misLen, 1.0, 0.0);

		free_mem(disparityMapRight);
		free_mem(disparityMapRightS);
		free_mem(disparityMapLeft);
		free_mem(refinedCostMatrixRight);
		free_mem(refinedCostMatrix);
		free_mem(costMatrix);
	}
}