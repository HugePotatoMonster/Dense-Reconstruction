#include "../../../include/StereoMapping/CostOptimizer/smCostOptimizer.h"

namespace StereoMapping {
	void CostOptimizer::smInternalConsistencyCheckF(f64* leftDisparityMap, f64* rightDisparityMap, f64* outputDisparityMap, u32 imageWidth, u32 imageHeight, f64 consistencyThreshold, f64 invalidPlaceholder) {
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				f64 leftDisp = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				i32 rightDispEpi = (i32)(i + leftDisp + 0.5);
				f64 rightDisp = 0;
				if (rightDispEpi < imageWidth) {
					rightDisp = get_pixel(rightDisparityMap, rightDispEpi, j, imageWidth, imageHeight);
				}
				f64 criteria = Abs(leftDisp - rightDisp);
				if (criteria <= consistencyThreshold) {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				}
				else {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
				}
			}
		}
	}
	void CostOptimizer::smDisparityMapDiscretization(f64* disparityMap, u32* outputMap, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32 invalidPlaceholder) {
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				get_pixel(outputMap, i, j, imageWidth, imageHeight) = get_pixel(disparityMap, i, j, imageWidth, imageHeight);
				if (get_pixel(disparityMap, i, j, imageWidth, imageHeight) > (f64)disparityRange) {
					get_pixel(outputMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
				}
			}
		}
	}
}