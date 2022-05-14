#pragma once
#include "cmTypeDefs.h"

namespace Common {
	namespace Math {
		class MathUtil {
		public:
			static void cmMat3Determinant(Mat3* inMat, OUT_ARG f64* outDet);
			static void cmMat3Inverse(Mat3* inMat, OUT_ARG Mat3* outMat);

			static void cmGetExtrinsicMatR(Common::Camera::MonocularCameraExtrinsic* camExt, OUT_ARG cv::Mat* outMat);
			static void cmGetExtrinsicMatT(Common::Camera::MonocularCameraExtrinsic* camExt, OUT_ARG cv::Mat* outMat);
			static void cmGetIntrinsicMat(Common::Camera::MonocularCameraIntrinsic* camInt, OUT_ARG cv::Mat* outMat);
			static void cmGetIntrinsicMatYFlip(Common::Camera::MonocularCameraIntrinsic* camInt,f64 h, OUT_ARG cv::Mat* outMat);
			static void cmGetVectorProductSkewSymMatrix(cv::Mat* inMat,OUT_ARG cv::Mat* outMat);
			static void cmGetRelativeExtrinsic(Common::Camera::MonocularCameraExtrinsic* camLeft, Common::Camera::MonocularCameraExtrinsic* camRight, Common::Camera::MonocularCameraExtrinsic* camRel);
		};
	}

}