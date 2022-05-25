#include "../../../include/SemiGlobalMatching/CostCalculator/smNullPathCostAggregator.h"
#include <iostream>
using namespace std;
namespace SemiGlobalMatching {
	void NullPathCostAggregator::smCostAggregate(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight,i32 minDisparity, u32 disparityRange, u32* refinedMatrix) {
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				for (i32 k = 0; k < (i32)disparityRange; k++) {
					get_pixel3(refinedMatrix, i, j, k, imageWidth, imageHeight, disparityRange) = get_pixel3(costMatrix, i, j, k, imageWidth, imageHeight, disparityRange);
				}
			}
		}
	}
}