#include "./include/Test/tsTest.h"
#include "./include/DepthEstimation/deDepthEstimationHelper.h"
#include "./include/Misc/msAuxiliaryUtils.h"
using namespace std;

int main() {

	//Reading intrinsic & extrinsic
	dbg_toutput("Read Parameters");
	cv::Mat camItr = cv::imread("E:\\60fps_images_archieve\\scene_00_0002.png", 0);
	cv::Mat camItl = cv::imread("E:\\60fps_images_archieve\\scene_00_0001.png", 0);
	Common::Camera::MonocularCameraIntrinsic camIntl, camIntr;
	Common::Camera::MonocularCameraExtrinsic camExtl, camExtr;
	Misc::AuxiliaryUtils::msParseIntrinsic("E:\\60fps_GT_archieve\\scene_00_0002.txt", &camIntr);
	Misc::AuxiliaryUtils::msParseExtrinsic("E:\\60fps_GT_archieve\\scene_00_0002.txt", &camExtr);
	Misc::AuxiliaryUtils::msParseIntrinsic("E:\\60fps_GT_archieve\\scene_00_0001.txt", &camIntl);
	Misc::AuxiliaryUtils::msParseExtrinsic("E:\\60fps_GT_archieve\\scene_00_0001.txt", &camExtl);
	
	//Depth Estimate
	dbg_toutput("Depth Estimate");
	DepthEstimation::DepthEstimationHelper* helper = new DepthEstimation::DepthEstimationHelper();
	cv::Mat reL, reR, RL, RR, PL, PR, Q;
	i32 flag = 0;
	f64* disparityMap = allocate_mem(f64, (usize)camItl.cols * camItl.rows);
	f64* depthMap = allocate_mem(f64, (usize)camItl.cols * camItl.rows);
	helper->deIdealCalibratedDepthEstimation(&camItl, &camItr, &camIntl, &camIntr, &camExtl, &camExtr, -64, 128,
		&reL, &reR, disparityMap, depthMap, &RL, &RR, &PL, &PR, &Q, &flag);

	//End
	dbg_toutput("Finished");
	delete helper;
	free_mem(disparityMap);
	free_mem(depthMap);
	return 0;
}