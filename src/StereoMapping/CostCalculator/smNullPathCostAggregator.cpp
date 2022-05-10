#include "../../../include/StereoMapping/CostCalculator/smNullPathCostAggregator.h"
#include <iostream>
using namespace std;
namespace StereoMapping {
	void NullPathCostAggregator::smCostAggregate(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix) {
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				for (i32 k = 0; k < disparityRange; k++) {
					get_pixel3(refinedMatrix, i, j, k, imageWidth, imageHeight, disparityRange) = get_pixel3(costMatrix, i, j, k, imageWidth, imageHeight, disparityRange);
				}
			}
		}
	}
}