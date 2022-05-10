#include "../../Common/cmTypeDefs.h"

namespace StereoMapping {
	class DisparityHelper {
	public:
		void smIdealBinocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight, u32 disparityRange, OUT_ARG f64* outputDisparity);
		void smIdealRandomMonocularDisparity(u8* imLeft, u8* imRight, u32 imageWidth, u32 imageHeight, u32 disparityRange)
	};
}