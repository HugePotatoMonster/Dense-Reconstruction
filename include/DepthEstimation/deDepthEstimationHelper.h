#pragma once
#include "../../include/Common/cmTypeDefs.h"
#include "../../include/DepthFilter/DepthFilter.h"
#include "../../include/DepthEstimation/deSemiGlobalMatchDepthEstimator.h"
//The interface for calling depth estimation functions
namespace DepthEstimation {
	class DepthEstimationHelper {
	public:
		// Rectify and estimate depth between image pairs that are recorded by ideal (no distortion) and calibrated (with known intrinsic and pose) camera
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
		// @param remapRotationL: (Output) The matrix that remaps coordinates in the unrectified C1 camera CS to rectified C1 camera CS.
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
		
		// Rectify, estimate depth and filter depth between image pairs that are recorded by ideal (no distortion) and calibrated (with known intrinsic and pose) camera
		// @author 1950641
		static void deIdealCalibratedDepthEstimationFilteredFromFile(std::string leftImage, std::string middleImage,
			std::string rightImage,std::string leftParam, std::string middleParam, std::string rightParam,
			OUT_ARG cv::Mat* leftDepth, OUT_ARG cv::Mat* leftExtrinsic, bool useFilter = false);
		



		//Depth Filter Utils
	private:
		static void transfer_depth(double* d_i, double* d, int iter, int flag, uint64_t imageWH)
		{
			if (flag == 1) // 从原来的depth中获取赋给depth_i
			{
				for (int i = 0; i < imageWH; i++)
				{
					d_i[i] = d[iter * imageWH + i];
				}
			}
			else // 将depth_i还给depth
			{
				for (int i = 0; i < imageWH; i++)
				{
					d[iter * imageWH + i] = d_i[i];
				}
			}
		}

		static double* use_filter(double* depthMap_left, double* depthMap_right, uint64_t imageWidth, uint64_t imageHeight, const cv::Mat left, const cv::Mat right) {
			using namespace cv;
			// dch: 初始化深度滤波器参数
			DepthFilter::FilterType filter_type = DepthFilter::Gaussion;
			static DepthFilter depth_filter;
			static double init_depth = 3.0;
			static double init_cov2 = 3.0;
			static std::once_flag m_flag;
			std::call_once(m_flag,
				[&imageWidth, &imageHeight,&filter_type]() {
					depth_filter.Initialize(imageWidth, imageHeight, init_depth, init_cov2, filter_type);
				}
			);

			printf("Updating Depth with Depth Dilter...\n");
			// 开始对每一张图进行深度滤波
			// 就一张图片了
			double* depth_a = new double[imageWidth * imageHeight]; // 存放每一张图片滤波后的深度信息

			// 选取后一张对前一张进行滤波 最后一张不动
			depth_filter.SetDepth(depthMap_left);
			depth_filter.UpdateDepth(depthMap_right, left, right); // 现在只更新一次
			depth_a = depth_filter.GetDepth();
			transfer_depth(depth_a, depthMap_left, 0, 2, imageWidth * imageHeight); // 滤波后更新到原来的depthMap中

			// 完成滤波
			printf("Depth Dilter Finished...\n");
			delete[] depth_a;
			return depthMap_left;
		}
	};
}