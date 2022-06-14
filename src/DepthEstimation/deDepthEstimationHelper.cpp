#include "../../include/DepthEstimation/deDepthEstimationHelper.h"
#include "../../include/Misc/msAuxiliaryUtils.h"
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
	void DepthEstimationHelper::deIdealCalibratedDepthEstimationFilteredFromFile(std::string leftImage, std::string middleImage, std::string rightImage,
		std::string leftParam, std::string middleParam, std::string rightParam,
		OUT_ARG cv::Mat* leftDepth, OUT_ARG cv::Mat* leftExtrinsic, bool useFilter) {

		//Reading intrinsic & extrinsic
		dbg_toutput("Read Parameters");
		cv::Mat camItr2 = cv::imread(rightImage, 0);
		cv::Mat camItr = cv::imread(middleImage,0);
		cv::Mat camItl = cv::imread(leftImage, 0);
		Common::Camera::MonocularCameraIntrinsic camIntl, camIntr, camIntr2;
		Common::Camera::MonocularCameraExtrinsic camExtl, camExtr, camExtr2;
		Misc::AuxiliaryUtils::msParseIntrinsic(rightParam, &camIntr2);
		Misc::AuxiliaryUtils::msParseExtrinsic(rightParam, &camExtr2);
		Misc::AuxiliaryUtils::msParseIntrinsic(middleParam, &camIntr);
		Misc::AuxiliaryUtils::msParseExtrinsic(middleParam, &camExtr);
		Misc::AuxiliaryUtils::msParseIntrinsic(leftParam, &camIntl);
		Misc::AuxiliaryUtils::msParseExtrinsic(leftParam, &camExtl);

		//Depth Estimate
		//Use `deIdealCalibratedDepthEstimation` is enough!
		dbg_toutput("Depth Estimate");

		//D1
		cv::Mat reL, reR, RL, RR, PL, PR, Q;
		i32 flag = 0;
		f64* disparityMap = allocate_mem(f64, (usize)camItl.cols * camItl.rows);
		f64* depthMap = allocate_mem(f64, (usize)camItl.cols * camItl.rows);
		deIdealCalibratedDepthEstimation(&camItl, &camItr, &camIntl, &camIntr, &camExtl, &camExtr, 0, 128,
			&reL, &reR, disparityMap, depthMap, &RL, &RR, &PL, &PR, &Q, &flag);

		//D2
		cv::Mat reL2, reR2, RL2, RR2, PL2, PR2, Q2;
		i32 flag2 = 0;
		f64* disparityMap2 = allocate_mem(f64, (usize)camItr.cols * camItr.rows);
		f64* depthMap2 = allocate_mem(f64, (usize)camItr.cols * camItr.rows);
		deIdealCalibratedDepthEstimation(&camItr, &camItr2, &camIntr, &camIntr2, &camExtr, &camExtr2, 0, 128,
			&reL2, &reR2, disparityMap2, depthMap2, &RL2, &RR2, &PL2, &PR2, &Q2, &flag2);
	
		//Depth Filter
		cv::Mat camExtlc, camExtrc, camExtr2c;
		Common::Math::MathUtil::cmGetExtrinsicMatA(&camExtl, &camExtlc);
		Common::Math::MathUtil::cmGetExtrinsicMatA(&camExtr, &camExtrc);
		Common::Math::MathUtil::cmGetExtrinsicMatA(&camExtr2, &camExtr2c);

		cv::Mat RL4,RL24;
		Common::Math::MathUtil::cmRot3ToRot4(&RL, &RL4);
		Common::Math::MathUtil::cmRot3ToRot4(&RL2, &RL24);

		cv::Mat camExtlcR = RL4 * camExtlc;
		cv::Mat camExtrcR = RL24 * camExtrc;
		if (useFilter) {
			use_filter(depthMap, depthMap2, camItl.cols, camItl.rows, camExtlcR, camExtlcR);
		}
		//Return
		memcpy(leftDepth->ptr<u8>(0), (void*)depthMap, sizeof(f64) * (camItr.cols * camItr.rows));
		for (i32 i = 0; i < 4; i++) {
			for (i32 j = 0; j < 4; j++) {
				if (j==3 && i<3){
					leftExtrinsic->at<f64>(i, j) = camExtlc.at<f64>(i, j)/1000;
				}
				else{
					leftExtrinsic->at<f64>(i, j) = camExtlc.at<f64>(i, j);
				}
				std::cout << leftExtrinsic->at<f64>(i, j) << ",";
			}
			std::cout << std::endl;
		}
	}
}