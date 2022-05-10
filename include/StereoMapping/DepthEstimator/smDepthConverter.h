#include "../../Common/cmTypeDefs.h"

namespace StereoMapping {
	class DepthConverter {
	public:
		void smIdealBinocularDisparityToDepth(f64* disparityMap, f64* depthMap, u32 imageWidth, u32 imageHeight, f64 cameraBaseline, f64 cameraFocalLength);
		void smDepthDiscretization(f64* depthMap, u32* outDepthMap,u32* outDepthMapMax, u32 imageWidth, u32 imageHeight);
	};
}