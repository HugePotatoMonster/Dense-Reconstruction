#include "../../Common/cmTypeDefs.h"

namespace StereoMapping {
	class CostOptimizer {
	public:
		void smInternalConsistencyCheckF(f64* leftDisparityMap, f64* rightDisparityMap,f64* outputDisparityMap, u32 imageWidth, u32 imageHeight,u32* occuList,u32* occuLen,u32* misList,u32* misLen, f64 consistencyThreshold = 1.0, f64 invalidPlaceholder = 1000.0);
		void smDisparityMapDiscretization(f64* disparityMap, u32* outputMap, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32 invalidPlaceholder = 0);
		void smExternalConsistencyCheckI(u32* leftDisparityMap, u32* rightDisparityMap, u32* outputDisparityMap, u32 imageWidth, u32 imageHeight, u32 consistencyThreshold = 1, u32 invalidPlaceholder = 255);
		void smConnectedBlockFiltering(f64* disparityMap, f64* outputMap, u32 imageWidth, u32 imageHeight, f64 discriminationThreshold = 1.0, u32 retainThreshold = 10);
		void smMedianFilter(f64* inputMap, f64* outputMap, u32 imageWidth, u32 imageHeight, u32 kSize);
		void smDisparityFill(f64* disparityMap, f64* outMap, u32 imageWidth, u32 imageHeight, u32* occuList, u32* occuLen, u32* misList, u32* misLen);
	};
}