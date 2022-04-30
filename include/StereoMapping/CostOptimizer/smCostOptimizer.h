#include "../../Common/cmTypeDefs.h"

namespace StereoMapping {
	class CostOptimizer {
	public:
		void smInternalConsistencyCheckF(f64* leftDisparityMap, f64* rightDisparityMap,f64* outputDisparityMap, u32 imageWidth, u32 imageHeight, f64 consistencyThreshold = 1.0, f64 invalidPlaceholder = 1000.0);
		void smDisparityMapDiscretization(f64* disparityMap, u32* outputMap, u32 disparityRange, u32 invalidPlaceholder = 0);
	};
}