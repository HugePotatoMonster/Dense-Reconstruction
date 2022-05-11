#include "../../../include/StereoMapping/Helper/smDisparityHelper.h"
namespace StereoMapping {
	void DisparityHelper::smAlgorithmConfigure(Constant::CostAggregationOption aggrOption, Constant::CostCalculationOption costOption) {
		this->aggrOption = aggrOption;
		this->costOption = costOption;
	}
	void DisparityHelper::smSetDisparityFill(i32 enable) {
		enableDisparityFill = enable;
	}
	void DisparityHelper::smDefineCostEstimator(CostCalculator** object) {
		if (this->costOption == Constant::CostCalculationOption::SMC_CENSUS_TRANSFORM) {
			*object = new CensusTransformCostCalculator();
		}
		else {
			*object = nullptr;
		}
	}
	void DisparityHelper::smDefineCostAggregator(CostAggregator** object) {
		if (this->aggrOption == Constant::CostAggregationOption::SMC_NONE) {
			*object = new NullPathCostAggregator();
		}else if (this->aggrOption == Constant::CostAggregationOption::SMC_FOUR_PATH) {
			*object = new FourPathCostAggregator();
		}
		else if (this->aggrOption == Constant::CostAggregationOption::SMC_EIGHT_PATH) {
			*object = new EightPathCostAggregator();
		}
		else {
			*object = nullptr;
		}
	}

	
	void DisparityHelper::smIdealBinocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight, u32 baselineLength, OUT_ARG f64* outputDisparityMap){
		// Disparity Estimation for Binocular Camera
		// Camera distortion is ignored
		CostCalculator* costEstimator;
		CostAggregator* costAggregator;
		CostOptimizer* costOptimizer = new CostOptimizer();
		smDefineCostEstimator(&costEstimator);
		smDefineCostAggregator(&costAggregator);
		if (costEstimator == nullptr || costAggregator == nullptr) {
			return;
		}
		u32 disparityRange = baselineLength;
		u8* costMatrix = allocate_mem(u8, (usize)imageWidth * imageHeight * disparityRange);
		costEstimator->smCostCalculate(imLeft, imRight, imageWidth, imageHeight, disparityRange, costMatrix);
		
		u32* refinedCostMatrix = allocate_mem(u32, (usize)imageWidth * imageHeight * disparityRange);
		set_zero(refinedCostMatrix, sizeof(u32) * imageWidth * imageHeight * disparityRange);
		costAggregator->smCostAggregate(imLeft, costMatrix, imageWidth, imageHeight, disparityRange, refinedCostMatrix);

		u32* refinedCostMatrixRight = allocate_mem(u32, (usize)imageWidth * imageHeight * disparityRange);
		costEstimator->smGetAnotherCost(refinedCostMatrix, imageWidth, imageHeight, disparityRange, refinedCostMatrixRight);

		f64* disparityMapLeft = allocate_mem(f64, (usize)imageWidth * imageHeight);
		f64* disparityMapLeftS = allocate_mem(f64, (usize)imageWidth * imageHeight);
		f64* disparityMapRight = allocate_mem(f64, (usize)imageWidth * imageHeight);
		f64* disparityMapRightS = allocate_mem(f64, (usize)imageWidth * imageHeight);
		u32* occlusionList = allocate_mem(u32, (usize)imageWidth * imageHeight);
		u32* mismatchList = allocate_mem(u32, (usize)imageWidth * imageHeight);
		u32 occlusionListLen = 0, mismatchListLen = 0;
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrix, disparityMapLeft, disparityMapLeftS, imageWidth, imageHeight, disparityRange);
		costEstimator->smDisparityEstimateSubpixelRefine<u32, f64>(refinedCostMatrixRight, disparityMapRight, disparityMapRightS, imageWidth, imageHeight, disparityRange);

		f64* disparityMapLeftAfter = allocate_mem(f64, (usize)imageWidth * imageHeight);
		costOptimizer->smInternalConsistencyCheckF(disparityMapLeft, disparityMapRight, disparityMapLeftAfter, imageWidth, imageHeight, occlusionList, &occlusionListLen, mismatchList, &mismatchListLen, 1.0, SGM_INVALID_DISPARITY_F);

		f64* leftDisparityMapCbc = allocate_mem(f64, (usize)imageWidth * imageHeight);
		costOptimizer->smConnectedBlockFiltering(disparityMapLeftAfter, leftDisparityMapCbc, imageWidth, imageHeight, 1.0, 50);
		
		if (this->enableDisparityFill) {
			f64* leftDisparityMapFl = allocate_mem(f64, (usize)imageWidth * imageHeight);
			costOptimizer->smDisparityFill(leftDisparityMapCbc, leftDisparityMapFl, imageWidth, imageHeight, occlusionList, &occlusionListLen, mismatchList, &mismatchListLen);
			costOptimizer->smMedianFilter(leftDisparityMapFl, outputDisparityMap, imageWidth, imageHeight, 3);
			free_mem(leftDisparityMapFl);
		}
		else {
			costOptimizer->smMedianFilter(leftDisparityMapCbc, outputDisparityMap, imageWidth, imageHeight, 3);
		}
		
		free_mem(leftDisparityMapCbc);
		free_mem(disparityMapLeftAfter);
		free_mem(mismatchList);
		free_mem(occlusionList);
		free_mem(disparityMapRightS);
		free_mem(disparityMapRight);
		free_mem(disparityMapLeftS);
		free_mem(disparityMapLeft);
		free_mem(refinedCostMatrixRight);
		free_mem(refinedCostMatrix);
		free_mem(costMatrix);
		delete costEstimator;
		delete costAggregator;
		delete costOptimizer;
	}
	void DisparityHelper::smIdealRandomMonocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight, u32 disparityRange, Common::Camera::MonocularCameraExtrinsic* leftPose, Common::Camera::MonocularCameraExtrinsic* rightPose, Common::Camera::MonocularCameraIntrinsic* camInt, OUT_ARG f64* outputDisparityMap) {
		// Disparity Estimation for Monocular Camera
		// Camera distortion is ignored
		// Pictures should be taken from different views

		// The pixel P=(u,v,1) on the left image corresponds to inv(I)P on the normalized camera CS
		// Then the coordinate on the camera plane is $z*inv(I)P$
		// Consider $z*inv(I)P=(R1)W+(t1)$, then the world coordinate W=inv(R1)[z*inv(I)P-(t1)]
		// 
		// L: inv(R1)[z*inv(I)P-(t1)]=0 is the spatial line, consider its projection related to the other camera
		// Camera coordinate C'=(R2)W+(t2) = (R2)inv(R1)[z*inv(I)P-(t1)]+(t2)
		// Homogeneous pixel coordiate P'=(F(z)U(z),F(z)V(z),F(z))=I(R2)inv(R1)[z*inv(I)P-(t1)]+I(t2)
		// 
		// Then, the line projected on the other camera is P'=(U(z),V(z))
		//

	}
}