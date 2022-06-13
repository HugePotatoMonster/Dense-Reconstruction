#include "../../../include/SemiGlobalMatching/DepthEstimator/smDepthConverter.h"

namespace SemiGlobalMatching {
	void DepthConverter::smIdealBinocularDisparityToDepth(f64* disparityMap, f64* depthMap, u32 imageWidth, u32 imageHeight, f64 cameraBaseline, f64 cameraFocalLength) {
		f64 ct = cameraBaseline * cameraFocalLength;
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				if (get_pixel(disparityMap, i, j, imageWidth, imageHeight) > EPS) {
					get_pixel(depthMap, i, j, imageWidth, imageHeight) = (cameraBaseline * cameraFocalLength) / get_pixel(disparityMap, i, j, imageWidth, imageHeight);
				}
				else {
					get_pixel(depthMap, i, j, imageWidth, imageHeight) = F64_INF;
				}
			}
		}
	}
	void DepthConverter::smDepthDiscretization(f64* depthMap, u32* outDepthMap, u32* outDepthMapMax, u32 imageWidth, u32 imageHeight) {
		u32 dMax = 0;
		f64 dMin = 1e90;
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				if (get_pixel(depthMap, i, j, imageWidth, imageHeight) <= DEPEST_INVALID_PIXEL + EPS) {
					continue;
				}
				dMin = Min(dMin, get_pixel(depthMap, i, j, imageWidth, imageHeight));
			}
		}
		dbg_toutput("DEPTH");
		dbg_toutput(dMin);
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				if (get_pixel(depthMap, i, j, imageWidth, imageHeight) <= DEPEST_INVALID_PIXEL + EPS) {
					get_pixel(outDepthMap, i, j, imageWidth, imageHeight) = 0;
					continue;
				}
				get_pixel(outDepthMap, i, j, imageWidth, imageHeight) = (u32)(get_pixel(depthMap, i, j, imageWidth, imageHeight) - dMin + 1.0);
				if (get_pixel(outDepthMap, i, j, imageWidth, imageHeight) > I32_MAX) {
					dbg_toutput("WARNING");
					dbg_toutput(get_pixel(depthMap, i, j, imageWidth, imageHeight));
					dbg_toutput(get_pixel(outDepthMap, i, j, imageWidth, imageHeight));
				}
				dMax = Max(dMax, get_pixel(outDepthMap, i, j, imageWidth, imageHeight));
			}
		}
		*outDepthMapMax = dMax;
	}
}