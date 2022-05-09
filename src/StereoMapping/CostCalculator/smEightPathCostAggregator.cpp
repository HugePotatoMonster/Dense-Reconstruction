#include "../../../include/StereoMapping/CostCalculator/smEightPathCostAggregator.h"
#include <iostream>
namespace StereoMapping {
	void EightPathCostAggregator::smCostAggregate(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix) {
		this->div = 8;
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
		i32 lastMin = I32_MAX;
		u32* optCost = allocate_mem(u32, disparityRange * 2);
		for (u32 i = 0; i < imageWidth ; i++) {
			lastMin = U32_MAX;
			//Current pixel is (?,j)
			i32 startCoord = direction ? imageHeight - 1 : 0;
			i32 stopCoord = direction ? -1 : imageHeight;
			i32 deltaCoord = direction ? -1 : 1;
			for (u32 k = 0; k < disparityRange * 2; k++) {
				optCost[k] = U32_MAX;
			}
			//Init Cond for Dynamic Programming
			for (u32 k = 0; k < disparityRange; k++) {
				get_pixel(optCost, k, startCoord & 1, disparityRange, 2) = get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange);
				lastMin = Min(lastMin, get_pixel(optCost, k, startCoord & 1, disparityRange, 2));
				get_pixel3(refinedMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) / div;
			}
			//Status Updating
			// k - vertical
			// q - horizontal
			for (i32 k = startCoord + deltaCoord , q = i; k != stopCoord; k += deltaCoord) {
				i32 newMin = I32_MAX;
				q += deltaCoord;
				q += (i32)imageWidth;
				q %= (i32)imageWidth;
				for (u32 d = 0; d < disparityRange; d++) {
					get_pixel(optCost, d, k & 1, disparityRange, 2) = get_pixel3(costMatrix, q, k, d, imageWidth, imageHeight, disparityRange);
					i32 addedValue = get_pixel(optCost, d, (k + 1) & 1, disparityRange, 2);
					if (d > 0) {
						addedValue = Min(addedValue, (i32)(get_pixel(optCost, d - 1, (k + 1) & 1, disparityRange, 2) + p1));
					}
					if (d < disparityRange - 1u) {
						addedValue = Min(addedValue, (i32)(get_pixel(optCost, d + 1, (k + 1) & 1, disparityRange, 2) + p1));
					}
					i32 p2Coef = p2 / (Abs((i32)get_pixel(imageData, q, k, imageWidth, imageHeight) - (i32)get_pixel(imageData, (q-deltaCoord+ (i32)imageWidth)% (i32)imageWidth, k - deltaCoord, imageWidth, imageHeight)) + 1);
					p2Coef = Max(p2Coef, (i32)p1);
					addedValue = Min(addedValue, (i32)(lastMin + p2Coef));
					get_pixel(optCost, d, k & 1, disparityRange, 2) += (addedValue - lastMin);
					newMin = Min(newMin, (i32)get_pixel(optCost, d, k & 1, disparityRange, 2));
					get_pixel3(refinedMatrix, q, k, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / div;
				}
				lastMin = newMin;
			}
		}
		free_mem(optCost);
	}
	void EightPathCostAggregator::smCostAggregateND(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix, u8 direction) {
		//Left Top -> Right Bottom
		i32 lastMin = I32_MAX;
		u32* optCost = allocate_mem(u32, disparityRange * 2);
		for (u32 i = 0; i < imageWidth; i++) {
			lastMin = U32_MAX;
			//Current pixel is (?,j)
			i32 startCoord = direction ? imageHeight - 1 : 0;
			i32 stopCoord = direction ? -1 : imageHeight;
			i32 deltaCoord = direction ? -1 : 1;
			for (u32 k = 0; k < disparityRange * 2; k++) {
				optCost[k] = U32_MAX;
			}
			//Init Cond for Dynamic Programming
			for (u32 k = 0; k < disparityRange; k++) {
				lastMin = Min(lastMin, get_pixel(optCost, k, startCoord & 1, disparityRange, 2) = get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange));
				get_pixel3(refinedMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) / div;
			}
			//Status Updating
			// k - vertical
			// q - horizontal
			for (i32 k = startCoord + deltaCoord, q = i; k != stopCoord; k += deltaCoord) {
				i32 newMin = I32_MAX;
				q -= deltaCoord;
				q += (i32)imageWidth;
				q %= (i32)imageWidth;
				for (u32 d = 0; d < disparityRange; d++) {
					get_pixel(optCost, d, k & 1, disparityRange, 2) = get_pixel3(costMatrix, q, k, d, imageWidth, imageHeight, disparityRange);
					i32 addedValue = get_pixel(optCost, d, (k + 1) & 1, disparityRange, 2);
					if (d > 0) {
						addedValue = Min(addedValue, (i32)(get_pixel(optCost, d - 1, (k + 1) & 1, disparityRange, 2) + p1));
					}
					if (d < disparityRange - 1u) {
						addedValue = Min(addedValue, (i32)(get_pixel(optCost, d + 1, (k + 1) & 1, disparityRange, 2) + p1));
					}
					i32 p2Coef = p2 / (Abs((i32)get_pixel(imageData, q, k, imageWidth, imageHeight) - (i32)get_pixel(imageData, (q + deltaCoord + (i32)imageWidth) % (i32)imageWidth, k - deltaCoord, imageWidth, imageHeight)) + 1);
					p2Coef = Max(p2Coef, (i32)p1);
					addedValue = Min(addedValue, (i32)(lastMin + p2Coef));
					get_pixel(optCost, d, k & 1, disparityRange, 2) += (addedValue - lastMin);
					newMin = Min(newMin, (i32)get_pixel(optCost, d, k & 1, disparityRange, 2));
					get_pixel3(refinedMatrix, q, k, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / div;
				}
				lastMin = newMin;
			}
		}
		free_mem(optCost);
	}
}