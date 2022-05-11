#include "../../Common/cmTypeDefs.h"
#include "../../StereoMapping/smDefs.h"
#include "../../../include/StereoMapping/Helper/smCostHelper.h"
#include "../../../include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"
#include "../../../include/StereoMapping/CostCalculator/smFourPathCostAggregator.h"
#include "../../../include/StereoMapping/CostCalculator/smEightPathCostAggregator.h"
#include "../../../include/StereoMapping/CostCalculator/smNullPathCostAggregator.h"
#include "../../../include/StereoMapping/CostOptimizer/smCostOptimizer.h"

namespace StereoMapping {
	class DisparityHelper {
	private:
		Constant::CostAggregationOption aggrOption = Constant::CostAggregationOption::SMC_EIGHT_PATH;
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
		void smIdealBinocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight, u32 baselineLength, OUT_ARG f64* outputDisparityMap);
		void smIdealRandomMonocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight, u32 disparityRange, Common::Camera::MonocularCameraExtrinsic* leftPose, Common::Camera::MonocularCameraExtrinsic* rightPose, Common::Camera::MonocularCameraIntrinsic* camInt , OUT_ARG f64* outputDisparityMap);
	};
}