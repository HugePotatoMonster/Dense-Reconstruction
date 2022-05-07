#include "../../../include/StereoMapping/CostCalculator/smEightPathCostAggregator.h"

namespace StereoMapping {
	void EightPathCostAggregator::smCostAggregate(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix) {
		smCostAggregateLR(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 0); //from left to right
		smCostAggregateLR(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 1); //from right to left
		smCostAggregateUD(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 0); //from top to bottom
		smCostAggregateUD(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 1); //from bottom to top
		smCostAggregatePD(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 0); //from top to bottom
		smCostAggregatePD(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 1); //from bottom to top
		smCostAggregateND(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 0); //from top to bottom
		smCostAggregateND(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 1); //from bottom to top
	}
	void EightPathCostAggregator::smCostAggregatePD(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix, u8 direction) {
		//Left Top -> Right Bottom
		u32 lastMin = U32_MAX;
		u32* optCost = allocate_mem(u32, disparityRange * 2);
		i32 deltaCoord;
		for (u32 i = 0; i < imageWidth + imageHeight; i++) {
			//Current pixel is (?,j)
			i32 startCoordX = 0, startCoordY = 0, stopCoordX = 0, stopCoordY = 0;
			i32 exceedCompensation = 0;
			if (i < imageHeight) {
				//Vertical => startCoord = (0,i)
				//startCoord = (imageHeight-1-i,imageHeight-1)
				if (direction) {
					startCoordX = imageHeight - 1 - i;
					startCoordY = imageHeight - 1;
					stopCoordX = 0;
					stopCoordY = i;
					deltaCoord = -1;
				}
				else {
					stopCoordX = imageHeight - 1 - i;
					stopCoordY = imageHeight - 1;
					startCoordX = 0;
					startCoordY = i;
					deltaCoord = 1;
				}
			}
			else {
				u32 ix = i - imageHeight;
				if (direction) {
					stopCoordX = ix;
					stopCoordY = 0;
					startCoordX = imageWidth - 1;
					startCoordY = imageWidth - 1 + ix;
					deltaCoord = -1;
				}
				else {
					startCoordX = ix;
					startCoordY = 0;
					stopCoordX = imageWidth - 1;
					stopCoordY = imageWidth - 1 + ix;
					deltaCoord = 1;
				}
			}
			for (u32 k = 0; k < disparityRange * 2; k++) {
				optCost[k] = U32_MAX;
			}
			u32 fw = 0;
			//Init Cond for Dynamic Programming
			for (u32 k = 0; k < disparityRange; k++) {
				get_pixel(optCost, k, fw, disparityRange, 2) = get_pixel3(costMatrix, startCoordX, startCoordY, k, imageWidth, imageHeight, disparityRange);
				//lastMin = Min(lastMin, );
				//get_pixel3(refinedMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange);
			}


			
		}
	}
	void EightPathCostAggregator::smCostAggregateND(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix, u8 direction) {

	}
}