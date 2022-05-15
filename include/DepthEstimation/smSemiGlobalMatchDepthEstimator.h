#pragma once
#include "../Common/cmTypeDefs.h"
#include "../Common/cmMath.h"
#include "smAbstractDepthEstimator.h"
#include "../SemiGlobalMatching/Helper/smDisparityHelper.h"
#include "../StereoRectification/srStereoRectification.h"

namespace DepthEstimation {
	class SemiGlobalMatchingDepthEstimator:public AbstractDepthEstimator {
	private:
		SemiGlobalMatching::DisparityHelper* disparityHelper = new SemiGlobalMatching::DisparityHelper();
		StereoRectification::AbstractStereoRectification* rectHelper = new StereoRectification::StereoRectification();
		
	public:
		virtual void deIdealCalibratedDisparityEstimation(cv::Mat* imLeft, cv::Mat* imRight, i32 minDisparity, i32 maxDisparity, Common::Camera::MonocularCameraIntrinsic* intLeft, Common::Camera::MonocularCameraIntrinsic* intRight, Common::Camera::MonocularCameraExtrinsic* extLeft, Common::Camera::MonocularCameraExtrinsic* extRight, OUT_ARG f64* disparityLeft, OUT_ARG cv::Mat* depthConversionMatrix, OUT_ARG i32* returnedFlag, OUT_ARG cv::Mat* extraRotationMatL, OUT_ARG cv::Mat* extraRotationMatR, OUT_ARG cv::Mat* extraProjMatL, OUT_ARG cv::Mat* extraProjMatR, OUT_ARG cv::Mat* maskL, OUT_ARG cv::Mat* maskR) override;
		virtual void deRectifiedDisparityEstimation(cv::Mat* imLeft, cv::Mat* imRight, i32 minDisparity, i32 maxDisparity, OUT_ARG f64* disparityLeft) override;
		~SemiGlobalMatchingDepthEstimator(){
			delete disparityHelper;
			delete rectHelper;
		}
	};
}