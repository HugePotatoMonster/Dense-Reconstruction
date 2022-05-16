#pragma once
#include "../../Common/cmTypeDefs.h"
#include "../../SemiGlobalMatching/smDefs.h"
#include "../../../include/SemiGlobalMatching/Helper/smCostHelper.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smCensusTransformCostCalculator.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smFourPathCostAggregator.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smEightPathCostAggregator.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smNullPathCostAggregator.h"
#include "../../../include/SemiGlobalMatching/CostOptimizer/smCostOptimizer.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smFourPathCostAggregatorCuda.h"
#include "../../../include/SemiGlobalMatching/CostCalculator/smEightPathCostAggregatorCuda.h"
namespace SemiGlobalMatching {
	class DisparityHelper {
	private:
		Constant::CostAggregationOption aggrOption = Constant::CostAggregationOption::SMC_EIGHT_PATH_CUDA;
		Constant::CostCalculationOption costOption = Constant::CostCalculationOption::SMC_CENSUS_TRANSFORM;
		i32 enableDisparityFill = false;

	private:
		void smDefineCostEstimator(CostCalculator** object);
		void smDefineCostAggregator(CostAggregator** object);

	public:					

		//Config
		void smAlgorithmConfigure(Constant::CostAggregationOption aggrOption, Constant::CostCalculationOption costOption);
		void smSetDisparityFill(i32 enable);
		//Function
		void smIdealBinocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight,i32 minDisparity, u32 baselineLength, OUT_ARG f64* outputDisparityMap);
	};
}