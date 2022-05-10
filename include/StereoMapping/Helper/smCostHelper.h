#include "../../Common/cmTypeDefs.h"

namespace StereoMapping {
	class CostHelper {
	public:
		//Test Interfaces
		void calculateCost(u8* imageLeft, u8* imageRight, u32 imageWidth, u32 imageHeight, u32 disparityRange,u32* leftDisparityMap);
		void calculateCostInternalF(u8* imageLeft, u8* imageRight, u32 imageWidth, u32 imageHeight, u32 disparityRange, f64* leftDisparityMap, f64* leftDisparitymapS, u32* occuList, u32* occuLen, u32* misList, u32* misLen);

	};
}