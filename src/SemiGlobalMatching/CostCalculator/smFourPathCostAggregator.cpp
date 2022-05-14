#include "../../../include/SemiGlobalMatching/CostCalculator/smFourPathCostAggregator.h"
#include <iostream>
using namespace std;
namespace SemiGlobalMatching {
	void FourPathCostAggregator::smCostAggregate(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix) {
		smCostAggregateLR(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 0); //from left to right
		smCostAggregateLR(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 1); //from right to left
		smCostAggregateUD(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 0); //from top to bottom
		smCostAggregateUD(imageData, costMatrix, imageWidth, imageHeight, disparityRange, refinedMatrix, 1); //from bottom to top
	}
	void FourPathCostAggregator::smCostAggregateLR(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix, u8 direction) {
		//Optimization problem: f[i,j,k] = C[i,j,k] + min(f[ix,jx,k],f[ix,jx,k+1]+p1,f[ix,jx,k-1]+p1,f[ix,jx,t]+p2)-min(f[ix,jx,t])
		//Array scrolling for "f" to reduce memory consumption
		u32 lastMin = U32_MAX;
		u32* optCost = allocate_mem(u32, disparityRange * 2);

		for (u32 j = 0; j < imageHeight; j++) {
			lastMin = U32_MAX;
			//Current pixel is (?,j)
			i32 startCoord = direction ? imageWidth - 1 : 0;
			i32 stopCoord = direction ? -1 : imageWidth;
			i32 deltaCoord = direction ? -1 : 1;
			for (u32 k = 0; k < disparityRange * 2; k++) {
				optCost[k] = U32_MAX;
			}
			//Init Cond for Dynamic Programming
			for (u32 k = 0; k < disparityRange; k++) {
				get_pixel(optCost, k, startCoord & 1, disparityRange, 2) = get_pixel3(costMatrix, startCoord, j, k, imageWidth, imageHeight, disparityRange);
				lastMin = Min(lastMin, get_pixel(optCost, k, startCoord & 1, disparityRange, 2));
				get_pixel3(refinedMatrix, startCoord, j, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, startCoord, j, k, imageWidth, imageHeight, disparityRange) / div;
			}
			//Status Updating
			for (i32 k = startCoord + deltaCoord; k != stopCoord; k += deltaCoord) {
				//Updating f[k,j,d] from f[k',j,d']
				i32 newMin = I32_MAX;
				for (u32 d = 0; d < disparityRange; d++) {
					get_pixel(optCost, d, k & 1, disparityRange, 2) = get_pixel3(costMatrix, k, j, d, imageWidth, imageHeight, disparityRange);
					i32 addedValue = get_pixel(optCost, d, (k + 1) & 1, disparityRange, 2);
					if (d > 0) {
						if ((i32)(get_pixel(optCost, d - 1, (k + 1) & 1, disparityRange, 2) + p1) < addedValue) {
							addedValue = get_pixel(optCost, d - 1, (k + 1) & 1, disparityRange, 2) + p1;
						}
					}
					if (d < disparityRange - 1) {
						if ((i32)(get_pixel(optCost, d + 1, (k + 1) & 1, disparityRange, 2) + p1) < addedValue) {
							addedValue = (get_pixel(optCost, d + 1, (k + 1) & 1, disparityRange, 2) + p1);
						}
					}
					i32 p2Coef = p2 / (Abs((i32)get_pixel(imageData, k, j, imageWidth, imageHeight) - (i32)get_pixel(imageData, k - deltaCoord, j, imageWidth, imageHeight)) + 1);
					p2Coef = Max(p2Coef, (i32)p1);
					if ((i32)(lastMin + p2Coef) < addedValue) {
						addedValue = Min(addedValue, (i32)(lastMin + p2Coef));
					}

					get_pixel(optCost, d, k & 1, disparityRange, 2) += (addedValue - lastMin);
					newMin = Min(newMin, (i32)get_pixel(optCost, d, k & 1, disparityRange, 2));
					get_pixel3(refinedMatrix, k, j, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / div;
				}
				lastMin = newMin;
			}
		}
		free_mem(optCost);

	}
	void FourPathCostAggregator::smCostAggregateUD(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix, u8 direction) {
		//Optimization problem: f[i,j,k] = C[i,j,k] + min(f[ix,jx,k],f[ix,jx,k+1]+p1,f[ix,jx,k-1]+p1,f[ix,jx,t]+p2)-min(f[ix,jx,t])
		//Array scrolling for "f" to reduce memory consumption
		u32 lastMin = U32_MAX;
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
			for (i32 k = startCoord + deltaCoord; k != stopCoord; k += deltaCoord) {
				//Updating f[k,j,d] from f[k',j,d']
				i32 newMin = I32_MAX;
				for (u32 d = 0; d < disparityRange; d++) {
					get_pixel(optCost, d, k & 1, disparityRange, 2) = get_pixel3(costMatrix, i, k, d, imageWidth, imageHeight, disparityRange);
					i32 addedValue = get_pixel(optCost, d, (k + 1) & 1, disparityRange, 2);
					if (d > 0) {
						addedValue = Min(addedValue, (i32)(get_pixel(optCost, d - 1, (k + 1) & 1, disparityRange, 2) + p1));
					}
					if (d < disparityRange - 1u) {
						addedValue = Min(addedValue, (i32)(get_pixel(optCost, d + 1, (k + 1) & 1, disparityRange, 2) + p1));
					}
					i32 p2Coef = p2 / (Abs((i32)get_pixel(imageData, i, k, imageWidth, imageHeight) - (i32)get_pixel(imageData, i, k - deltaCoord, imageWidth, imageHeight)) + 1);
					p2Coef = Max(p2Coef, (i32)p1);
					addedValue = Min(addedValue, (i32)(lastMin + p2Coef));
					get_pixel(optCost, d, k & 1, disparityRange, 2) += (addedValue - lastMin);
					newMin = Min(newMin, (i32)get_pixel(optCost, d, k & 1, disparityRange, 2));
					get_pixel3(refinedMatrix, i, k, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / div;
				}
				lastMin = newMin;
			}
		}
		free_mem(optCost);
	}

}