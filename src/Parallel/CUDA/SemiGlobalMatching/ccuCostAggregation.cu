#include "../../../../include/Common/cmTypeDefs.h"
#include "../../../../include/Parallel/CUDA/ccuDeclarations.h"
namespace Parallel {
	namespace CUDA {
		namespace SemiGlobalMatching {
			namespace CostAggregator {
				cu_global void cusmParallelCostAggregationLR(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR) {
					f64 p1 = 10,p2=150;
					u32 lastMin = U32_MAX;
					i32 parallel = threadIdx.x;
					u32* optCost = optCostR + (2 * disparityRange * parallel);
					for (u32 j = parallel; j < imageHeight; j+=threads) {
						lastMin = U32_MAX;
						i32 startCoord = direction ? imageWidth - 1 : 0;
						i32 stopCoord = direction ? -1 : imageWidth;
						i32 deltaCoord = direction ? -1 : 1;
						for (u32 k = 0; k < disparityRange * 2; k++) {
							optCost[k] = U32_MAX;
						}
						for (u32 k = 0; k < disparityRange; k++) {
							get_pixel(optCost, k, startCoord & 1, disparityRange, 2) = get_pixel3(costMatrix, startCoord, j, k, imageWidth, imageHeight, disparityRange);
							lastMin = Min(lastMin, get_pixel(optCost, k, startCoord & 1, disparityRange, 2));
							get_pixel3(refinedMatrix, startCoord, j, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, startCoord, j, k, imageWidth, imageHeight, disparityRange) / 8;
						}
						for (i32 k = startCoord + deltaCoord; k != stopCoord; k += deltaCoord) {
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
								get_pixel3(refinedMatrix, k, j, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / 8;
							}
							lastMin = newMin;
						}
					}
				}
				cu_global void cusmParallelCostAggregationUD(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR) {
					f64 p1 = 10,p2=150;
					u32 lastMin = U32_MAX;
					i32 parallel = threadIdx.x;
					u32* optCost = optCostR + (2 * disparityRange * parallel);
					for (u32 i = parallel; i < imageWidth; i+=threads) {
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
							get_pixel3(refinedMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) / 8;
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
								get_pixel3(refinedMatrix, i, k, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / 8;
							}
							lastMin = newMin;
						}
					}
				}
				cu_global void cusmParallelCostAggregationND(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR) {
					f64 p1 = 10,p2=150;
					u32 lastMin = U32_MAX;
					i32 parallel = threadIdx.x;
					u32* optCost = optCostR + (2 * disparityRange * parallel);
					for (u32 i = parallel; i < imageWidth; i+=threads) {
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
							lastMin = Min(lastMin, (i32)(get_pixel(optCost, k, startCoord & 1, disparityRange, 2) = get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange)));
							get_pixel3(refinedMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) / 8;
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
								get_pixel3(refinedMatrix, q, k, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / 8;
							}
							lastMin = newMin;
						}
					}
				}
				cu_global void cusmParallelCostAggregationPD(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR) {
					//printf("I am In UD\n");
					f64 p1 = 10,p2=150;
					u32 lastMin = U32_MAX;
					i32 parallel = threadIdx.x;
					u32* optCost = optCostR + (2 * disparityRange * parallel);
					for (u32 i = parallel; i < imageWidth; i+=threads) {
						lastMin = U32_MAX;
						i32 startCoord = direction ? imageHeight - 1 : 0;
						i32 stopCoord = direction ? -1 : imageHeight;
						i32 deltaCoord = direction ? -1 : 1;
						for (u32 k = 0; k < disparityRange * 2; k++) {
							optCost[k] = U32_MAX;
						}
						for (u32 k = 0; k < disparityRange; k++) {
							get_pixel(optCost, k, startCoord & 1, disparityRange, 2) = get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange);
							lastMin = Min(lastMin, (i32)get_pixel(optCost, k, startCoord & 1, disparityRange, 2));
							get_pixel3(refinedMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) += get_pixel3(costMatrix, i, startCoord, k, imageWidth, imageHeight, disparityRange) / 8;
						}
						for (i32 k = startCoord + deltaCoord, q = i; k != stopCoord; k += deltaCoord) {
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
								i32 p2Coef = p2 / (Abs((i32)get_pixel(imageData, q, k, imageWidth, imageHeight) - (i32)get_pixel(imageData, (q - deltaCoord + (i32)imageWidth) % (i32)imageWidth, k - deltaCoord, imageWidth, imageHeight)) + 1);
								p2Coef = Max(p2Coef, (i32)p1);
								addedValue = Min(addedValue, (i32)(lastMin + p2Coef));
								get_pixel(optCost, d, k & 1, disparityRange, 2) += (addedValue - lastMin);
								newMin = Min(newMin, (i32)get_pixel(optCost, d, k & 1, disparityRange, 2));
								get_pixel3(refinedMatrix, q, k, d, imageWidth, imageHeight, disparityRange) += get_pixel(optCost, d, k & 1, disparityRange, 2) / 8;
							}
							lastMin = newMin;
						}
					}
				}
				void cusmParallelCostAggregationFourPathCaller(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, u32* optCost) {
					//Copy Memory to GPU
					u8* imageDataCu = nullptr;
					u32* costMatrixCu = nullptr;
					u32* refinedMatrixCu = nullptr;
					u32* optCostF = new u32[disparityRange * 2];
					set_zero(optCostF, sizeof(u32) * 2 * disparityRange);
					u32* optCostCu = nullptr;
					cudaMalloc((void**)&imageDataCu, sizeof(u8) * (usize)imageWidth * imageHeight);
					cudaMemcpy(imageDataCu, imageData, sizeof(u8) * (usize)imageWidth * imageHeight, cudaMemcpyHostToDevice);
					cudaMalloc((void**)&costMatrixCu, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange);
					cudaMemcpy(costMatrixCu, costMatrix, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange, cudaMemcpyHostToDevice);
					cudaMalloc((void**)&refinedMatrixCu, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange);
					cudaMalloc((void**)&optCostCu, sizeof(u32) * (usize)disparityRange * 2);
					//Set Zero
					cudaMemcpy(refinedMatrixCu, refinedMatrix, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange, cudaMemcpyHostToDevice);
					cudaMemcpy(optCostCu, optCostF, sizeof(u32) * (usize)disparityRange * 2, cudaMemcpyHostToDevice);

					//Then Cost Aggregation Starts
					//TODO: Bug Fix
					/*
					cusmParallelCostAggregationLR << <1, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 0, threads, optCostCu);
					cu_sync();
					cusmParallelCostAggregationLR << <1, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 1, threads, optCostCu);
					cu_sync();
					cusmParallelCostAggregationUD << <1, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 0, threads, optCostCu);
					cu_sync();
					cusmParallelCostAggregationUD << <1, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 1, threads, optCostCu);
					cu_sync();*/
					
					//Return Value
					cudaMemcpy(refinedMatrix, refinedMatrixCu, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange, cudaMemcpyDeviceToHost);
					//Free GPU
					cudaFree(imageDataCu);
					cudaFree(refinedMatrixCu);
					cudaFree(costMatrixCu);
					cudaFree(optCostCu);
				}
				void cusmParallelCostAggregationEightPathCaller(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, u32* optCost) {
					//Copy Memory to GPU
					
					u8* imageDataCu = nullptr;
					u32* costMatrixCu = nullptr;
					u32* refinedMatrixCu = nullptr;
					u32* optCostCu = nullptr;
					cudaMalloc((void**)&imageDataCu, sizeof(u8) * (usize)imageWidth * imageHeight);
					cudaMemcpy(imageDataCu, imageData, sizeof(u8) * (usize)imageWidth * imageHeight, cudaMemcpyHostToDevice);
					cudaMalloc((void**)&costMatrixCu, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange);
					cudaMemcpy(costMatrixCu, costMatrix, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange, cudaMemcpyHostToDevice);
					cudaMalloc((void**)&refinedMatrixCu, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange);
					cudaMalloc((void**)&(optCostCu), sizeof(u32) * (usize)disparityRange * 2 * threads );
					usize offset = sizeof(u32) * (usize)disparityRange * 2 * threads;
					//Set Zero
					cudaMemcpy(refinedMatrixCu, refinedMatrix, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange, cudaMemcpyHostToDevice);
					printf("STARTS");
					//Then Cost Aggregation Starts
					cusmParallelCostAggregationLR << <1, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 0, threads, optCostCu);
					cusmParallelCostAggregationLR << <2, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 1, threads, optCostCu + offset * 1);
					cusmParallelCostAggregationUD << <3, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 0, threads, optCostCu + offset * 2);
					cusmParallelCostAggregationUD << <4, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 1, threads, optCostCu + offset * 3);
					cusmParallelCostAggregationND << <5, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 0, threads, optCostCu + offset * 4);
					cusmParallelCostAggregationND << <6, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 1, threads, optCostCu + offset * 5);
					cusmParallelCostAggregationPD << <7, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 0, threads, optCostCu + offset * 6);
					cusmParallelCostAggregationPD << <8, threads >> > (imageDataCu, costMatrixCu, imageWidth, imageHeight, minDisparity, disparityRange, refinedMatrixCu, 1, threads, optCostCu + offset * 7);
					cu_sync();
					printf("ENDS");
					//Return Value
					cudaMemcpy(refinedMatrix, refinedMatrixCu, sizeof(u32) * (usize)imageWidth * imageHeight * disparityRange, cudaMemcpyDeviceToHost);
					printf("%d ", get_pixel3(refinedMatrix, 14, 14, 0, imageWidth, imageHeight, disparityRange));
					printf("%d ", get_pixel3(refinedMatrix, 14, 14, 63, imageWidth, imageHeight, disparityRange));
					printf("%d ", get_pixel3(refinedMatrix, 14, 14, 32, imageWidth, imageHeight, disparityRange));

					//Free GPU
					cudaFree(imageDataCu);
					cudaFree(refinedMatrixCu);
					cudaFree(costMatrixCu);
					cudaFree(optCostCu);
				}
			}
		}
	}
}