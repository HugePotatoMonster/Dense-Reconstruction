#include "../../include/DepthEstimation/deGeneralDepthEstimator.h"

namespace DepthEstimation {
	void GeneralDepthEstimator::deGeneralDisparityToDepth(f64* disparityMap, u32 imageWidth, u32 imageHeight, cv::Mat* conversionMatrix, OUT_ARG f64* depthMap) {
		cv::Mat pixel = (cv::Mat_<f64>(4, 1) << 0, 0, 0, 1);
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				if (get_pixel(disparityMap, i, j, imageWidth, imageHeight) > SGM_INVALID_DISPARITY_F_THRESH + EPS) {
					get_cvmat(pixel, 0, 0) = i;
					get_cvmat(pixel, 1, 0) = j;
					get_cvmat(pixel, 2, 0) = get_pixel(disparityMap, i, j, imageWidth, imageHeight);
					cv::Mat voxel = *conversionMatrix * pixel;
					//Normalize the homogeneous coordinate
					get_pixel(depthMap, i, j, imageWidth, imageHeight) = get_cvmat(voxel, 2, 0) / get_cvmat(voxel, 3, 0);
					if(get_pixel(disparityMap, i, j, imageWidth, imageHeight)<67 || i<128){
						get_pixel(depthMap, i, j, imageWidth, imageHeight) = 1e30;
					}
				}
				else {
					//Handle the invalid disparity
					get_pixel(depthMap, i, j, imageWidth, imageHeight) = DEPEST_INVALID_PIXEL;
				}
			}
		}
	}
}
