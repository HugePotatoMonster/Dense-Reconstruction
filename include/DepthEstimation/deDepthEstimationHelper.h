#pragma once
#include "../../include/Common/cmTypeDefs.h"
#include "../../include/DepthEstimation/deSemiGlobalMatchDepthEstimator.h"
//The interface for calling depth estimation functions
namespace DepthEstimation {
	class DepthEstimationHelper {
	public:
		// Rectify and estimate depth between image pairs that are recorded by ideal (no distortion) and calibrated (with known intrinsic and pose)
		// @author 1950641
		// @param imLeft: (Input) Image taken from the left camera C1. Type should be CV_8UC1 (That is, the intensity map)
		// @param imRight: (Input) Image taken from the right camera C2. Type should be in CV_8UC1 (That is, the intensity map)
		// @param leftInt: (Input) Intrinsic for C1
		// @param rightInt: (Input) Intrinsic for C2
		// @param leftExt: (Input) Extrinsic for C1
		// @param rightExt: (Input) Extrinsic for C2
		// @param minDisparity: (Input) The possible lower bound for disparity in the rectified pair
		// @param maxDisparity: (Input) The possible upper bound for disparity in the rectified pair
		// @param rectifiedImLeft: (Output) Rectified left image
		// @param rectifiedImRight: (Output) Rectified right image
		// @param disparityMap: (Output) The pointer to a 1D array that pixel-wise disparity values will be stored into
		// @param depthMap: (Output) The pointer to a 1D array that pixel-wise depth values will be stored into. The depth value assumes the rectified first camera CS as the world coordinate.
		// @param remapRotatonL: (Output) The matrix that remaps coordinates in the unrectified C1 camera CS to rectified C1 camera CS.
		// @param remapRotationR: (Output) The matrix that remaps coordinates in the unrectified C2 camera CS to rectified C2 camera CS.
		// @param rectifiedProjectionL: (Output) The matrix that gives projection from rectified C1 camera CS to rectified C1 image/pixel CS. (Namely, the new intrinsic for C1)
		// @param rectifiedProjectionR: (Output) The matrix that gives projection from rectified C1 camera CS to rectified C2 image/pixel CS.
		// @param disparityToDepthMap: (Output) The 4x4 matrix that projects pixel-wise disparity from rectified left image to the rectified first camera CS
		// @param returnFlag: (Output) Reserved
		static void deIdealCalibratedDepthEstimation(cv::Mat* imLeft, cv::Mat* imRight,
			Common::Camera::MonocularCameraIntrinsic* leftInt, Common::Camera::MonocularCameraIntrinsic* rightInt,
			Common::Camera::MonocularCameraExtrinsic* leftExt, Common::Camera::MonocularCameraExtrinsic* rightExt,
			i32 minDisparity, i32 maxDisparity, OUT_ARG cv::Mat* rectifiedImLeft, OUT_ARG cv::Mat* rectifiedImRight,
			OUT_ARG f64* disparityMap, OUT_ARG f64* depthMap, OUT_ARG cv::Mat* remapRotationL, OUT_ARG cv::Mat* remapRotationR,
			OUT_ARG cv::Mat* rectifiedProjectionL, OUT_ARG cv::Mat* rectifiedProjectionR, OUT_ARG cv::Mat* disparityToDepthMat, OUT_ARG i32* returnFlag);
	};
}