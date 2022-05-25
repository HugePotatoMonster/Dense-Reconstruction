#include "../../include/DepthEstimation/deSemiGlobalMatchDepthEstimator.h"

namespace DepthEstimation {

	void SemiGlobalMatchingDepthEstimator::deIdealCalibratedDisparityEstimation(cv::Mat* imLeft, cv::Mat* imRight,
		i32 minDisparity, i32 maxDisparity,
		Common::Camera::MonocularCameraIntrinsic* intLeft, Common::Camera::MonocularCameraIntrinsic* intRight,
		Common::Camera::MonocularCameraExtrinsic* extLeft, Common::Camera::MonocularCameraExtrinsic* extRight,
		OUT_ARG f64* disparityLeft, OUT_ARG cv::Mat* depthConversionMatrix,
		OUT_ARG i32* returnedFlag, OUT_ARG cv::Mat* extraRotationMatL, OUT_ARG cv::Mat* extraRotationMatR,
		OUT_ARG cv::Mat* extraProjMatL, OUT_ARG cv::Mat* extraProjMatR, OUT_ARG cv::Mat* maskL, OUT_ARG cv::Mat* maskR) {
		//Rectify and calculate disparity for a calibrated stereo pair
		//Distortion is ignored
		cv::Mat rimLeft, rimRight;
		cv::Mat homoLeft, homoRight;
		
		rectHelper->stereoRectify(imLeft, imRight, intLeft, intRight, extLeft, extRight, &rimLeft, &rimRight, maskL, maskR, &homoLeft, &homoRight,
			extraRotationMatL, extraRotationMatR, extraProjMatL, extraProjMatR, depthConversionMatrix);
		disparityHelper->smIdealBinocularDisparity(rimLeft.data, rimRight.data, rimLeft.cols, rimRight.rows, minDisparity, maxDisparity, disparityLeft);
	}
	void SemiGlobalMatchingDepthEstimator::deRectifiedDisparityEstimation(cv::Mat* imLeft, cv::Mat* imRight,
		i32 minDisparity, i32 maxDisparity, OUT_ARG f64* disparityLeft) {
		//Calculate disparity for a  rectified stereo pair
		disparityHelper->smIdealBinocularDisparity(imLeft->data, imRight->data, imLeft->cols, imRight->cols, minDisparity, (u32)(maxDisparity - minDisparity), disparityLeft);
	}
}