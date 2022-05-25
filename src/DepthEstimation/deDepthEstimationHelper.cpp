#include "../../include/DepthEstimation/deDepthEstimationHelper.h"
namespace DepthEstimation {
	void DepthEstimationHelper::deIdealCalibratedDepthEstimation(cv::Mat* imLeft, cv::Mat* imRight,
		Common::Camera::MonocularCameraIntrinsic* leftInt, Common::Camera::MonocularCameraIntrinsic* rightInt,
		Common::Camera::MonocularCameraExtrinsic* leftExt, Common::Camera::MonocularCameraExtrinsic* rightExt,
		i32 minDisparity, i32 maxDisparity, OUT_ARG cv::Mat* rectifiedImLeft, OUT_ARG cv::Mat* rectifiedImRight,
		OUT_ARG f64* disparityMap, OUT_ARG f64* depthMap, OUT_ARG cv::Mat* remapRotationL, OUT_ARG cv::Mat* remapRotationR,
		OUT_ARG cv::Mat* rectifiedProjectionL, OUT_ARG cv::Mat* rectifiedProjectionR, OUT_ARG cv::Mat* disparityToDepthMat, OUT_ARG i32* returnFlag) {

		//Pre-checking
		dbg_trace(
			pr_assert_notnull(imLeft);
			pr_assert_notnull(imRight);
			pr_assert_notnull(leftInt);
			pr_assert_notnull(rightInt);
			pr_assert_notnull(leftExt);
			pr_assert_notnull(rightExt);
			pr_assert(minDisparity < maxDisparity);
			pr_assert_notnull(rectifiedImLeft);
			pr_assert_notnull(rectifiedImRight);
			pr_assert_notnull(disparityMap);
			pr_assert_notnull(depthMap);
			pr_assert_notnull(remapRotationL);
			pr_assert_notnull(remapRotationR);
			pr_assert_notnull(rectifiedProjectionL);
			pr_assert_notnull(rectifiedProjectionR);
			pr_assert_notnull(disparityToDepthMat);
			pr_assert_notnull(returnFlag);
			pr_assert(imLeft->cols == imRight->cols);
			pr_assert(imLeft->rows == imRight->rows);
			pr_assert(imLeft->type() == CV_8UC1);
			pr_assert(imRight->type() == CV_8UC1);
			pr_assert(imLeft->cols > 0);
			pr_assert(imLeft->rows > 0);
		)

		//Perform Rectification & Stereo Matching
		i32 estimationFlag = 0;
		cv::Mat maskL, maskR;
		AbstractDepthEstimator* estimator = new SemiGlobalMatchingDepthEstimator();
		estimator->deIdealCalibratedDisparityEstimation(imLeft, imRight, minDisparity, maxDisparity, leftInt, rightInt, leftExt, rightExt, disparityMap, disparityToDepthMat,
			&estimationFlag, remapRotationL, remapRotationR, rectifiedProjectionL, rectifiedProjectionR, &maskL, &maskR);

		//Perform Depth Estimation
		estimator->deGeneralDisparityToDepth(disparityMap, imLeft->cols, imLeft->rows, disparityToDepthMat, depthMap);
		delete estimator;
	}
}