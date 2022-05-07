#include "../../Common/cmTypeDefs.h"

namespace StereoMapping {
	class DepthConverter {
	public:
		void smBinocularDisparityToDepth(f64* disparityMap, f64* depthMap, u32 imageWidth, u32 imageHeight, u32 cameraBaseline, u32 cameraFocalLength);
	};
}