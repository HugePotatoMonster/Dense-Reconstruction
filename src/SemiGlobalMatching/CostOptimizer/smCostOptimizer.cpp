#include "../../../include/SemiGlobalMatching/CostOptimizer/smCostOptimizer.h"
#include <algorithm>
#include <iostream>

namespace SemiGlobalMatching {
	void CostOptimizer::smInternalConsistencyCheckF(f64* leftDisparityMap, f64* rightDisparityMap, f64* outputDisparityMap, u32 imageWidth, u32 imageHeight, u32* occuList, u32* occuLen, u32* misList, u32* misLen, f64 consistencyThreshold, f64 invalidPlaceholder) {
		for (i32 j = 0; j < (i32)imageHeight; j++) {
			for (i32 i = 0; i < (i32)imageWidth; i++) {
				f64 leftDisp = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				i32 rightDispEpi = (i32)(i - leftDisp + 0.5);
				f64 rightDisp = 0;
				if (rightDispEpi < (i32)imageWidth && rightDispEpi >= 0) {
					rightDisp = get_pixel(rightDisparityMap, rightDispEpi, j, imageWidth, imageHeight);
					f64 criteria = Abs(leftDisp - rightDisp);
					if (criteria <= consistencyThreshold) {
						get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
					}
					else {
						get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
						i32 leftDispRi = (i32)(rightDispEpi + rightDisp + 0.5);
						if (leftDispRi >= 0 && leftDispRi < (i32)imageWidth) {
							f64 leftDispR = get_pixel(leftDisparityMap, leftDispRi, j, imageWidth, imageHeight);
							if (leftDispR > leftDisp) {
								occuList[(*occuLen)++] = coord2idx(i, j, imageWidth, imageHeight);
							}
							else {
								misList[(*misLen)++] = coord2idx(i, j, imageWidth, imageHeight);
							}
						}
						else {
							misList[(*misLen)++] = coord2idx(i, j, imageWidth, imageHeight);
						}
					}
				}
				else {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
					misList[(*misLen)++] = coord2idx(i, j, imageWidth, imageHeight);
				}
			}
		}
	}
	void CostOptimizer::smDisparityMapDiscretization(f64* disparityMap, u32* outputMap, u32 imageWidth, u32 imageHeight,i32 minDisparity, u32 disparityRange, u32 invalidPlaceholder) {
		//TODO: minDisparity
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				get_pixel(outputMap, i, j, imageWidth, imageHeight) = (u32)(get_pixel(disparityMap, i, j, imageWidth, imageHeight)+ (f64)minDisparity);
				if (get_pixel(disparityMap, i, j, imageWidth, imageHeight) > (f64)disparityRange || get_pixel(disparityMap, i, j, imageWidth, imageHeight) < SGM_INVALID_DISPARITY_F + EPS) {
					get_pixel(outputMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
				}
			}
		}
	}
	void CostOptimizer::smExternalConsistencyCheckI(u32* leftDisparityMap, u32* rightDisparityMap, u32* outputDisparityMap, u32 imageWidth, u32 imageHeight, u32 consistencyThreshold, u32 invalidPlaceholder) {
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				u32 leftDisp = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				u32 rightDisp = get_pixel_hf(rightDisparityMap, i, j, imageWidth, imageHeight);
				u32 criteria = (u32)Abs((i32)leftDisp - (i32)rightDisp);
				if (criteria <= consistencyThreshold) {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				}
				else {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
				}
			}
		}
	}
	void CostOptimizer::smConnectedBlockFiltering(f64* disparityMap,f64* outputMap, u32 imageWidth, u32 imageHeight, f64 discriminationThreshold, u32 retainThreshold) {
		u8* visState = allocate_mem(u8, (usize)imageWidth * imageHeight);
		u32* bfsQueue = allocate_mem(u32, (usize)imageWidth * imageHeight);
		i32 bfsQueueRear = 0;
		i32 bfsQueueFront = 0;
		f64 curPixel = 0;
		const i32 bfsQueueLen = imageWidth * imageHeight;
		set_zero(visState, sizeof(u8) * imageWidth * imageHeight);
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				get_pixel(outputMap, i, j, imageWidth, imageHeight) = get_pixel(disparityMap, i, j, imageWidth, imageHeight);
			}
		}

		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				if (!get_pixel(visState, i, j, imageWidth, imageHeight)) {
					//Start BFS here
					//Re-init queue ptr
					bfsQueueRear = 0;
					bfsQueueFront = 0;
					queue_push(bfsQueue, coord2idx(i, j, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen);
					curPixel = disparityMap[coord2idx(i, j, imageWidth, imageHeight)];
					while (!is_queue_empty(bfsQueueFront, bfsQueueRear, bfsQueueLen)) {
						i32 top = queue_front(bfsQueue, bfsQueueFront, bfsQueueRear, bfsQueueLen);
						queue_pop(bfsQueue, bfsQueueFront, bfsQueueRear, bfsQueueLen);
						i32 tx = idx2xcoord(top, imageWidth, imageHeight);
						i32 ty = idx2ycoord(top, imageWidth, imageHeight);
						if (tx > 0) {
							if (Fabs(get_pixel(disparityMap, tx - 1, ty, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx - 1, ty, imageWidth, imageHeight)) {
									get_pixel(visState, tx - 1, ty, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx - 1, ty, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen);
								}
							}
						}
						if (ty > 0 && tx >= 0 && tx < (i32)imageWidth && ty < (i32)imageHeight) {
							if (Fabs(get_pixel(disparityMap, tx, ty - 1, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx, ty - 1, imageWidth, imageHeight)) {
									get_pixel(visState, tx, ty - 1, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx, ty - 1, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen);
								}
							}
						}
						if (tx < (i32)imageWidth - 1 && ty < (i32)imageHeight) {
							if (Fabs(get_pixel(disparityMap, tx + 1, ty, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx + 1, ty, imageWidth, imageHeight)) {
									get_pixel(visState, tx + 1, ty, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx + 1, ty, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen);
								}
							}
						}

						if (tx < (i32)imageWidth && ty < (i32)imageHeight - 1) {
							if (Fabs(get_pixel(disparityMap, tx, ty + 1, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx, ty + 1, imageWidth, imageHeight)) {
									get_pixel(visState, tx, ty + 1, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx, ty + 1, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen);
								}
							}
						}
					}
					if (bfsQueueRear < (i32)retainThreshold) {
						for (i32 i = 0; i < bfsQueueRear; i++) {
							outputMap[bfsQueue[i]] = SGM_INVALID_DISPARITY_F;
						}
					}
				}
			}
		}

		free_mem(visState);
		free_mem(bfsQueue);
	}
	void CostOptimizer::smMedianFilter(f64* inputMap, f64* outputMap, u32 imageWidth, u32 imageHeight, u32 kSize) {
		f64* medianList = allocate_mem(f64, (usize)kSize * kSize);
		i32 medianListLen = 0;
		i32 radius = ((i32)(kSize - 1)) / 2;
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				medianListLen = 0;
				for (i32 dx = -radius; dx < radius; dx++) {
					for (i32 dy = -radius; dy < radius; dy++) {
						if ((dx + i) >= 0 && (dx + i) < (i32)imageWidth) {
							if ((dy + j) >= 0 && (dy + j) < (i32)imageHeight) {
								medianList[medianListLen++] = get_pixel(inputMap, dx + i, dy + j, imageWidth, imageHeight);
							}
						}
					}
				}
				std::sort(medianList, medianList + medianListLen);
				get_pixel(outputMap, i, j, imageWidth, imageHeight) = medianList[medianListLen / 2];

			}
		}
		free_mem(medianList);
	}
	void CostOptimizer::smDisparityFill(f64* disparityMap, f64* outMap, u32 imageWidth, u32 imageHeight, u32* occuList, u32* occuLen, u32* misList, u32* misLen) {
		const i32 dirs = 8;
		const f64 invIdx = SGM_INVALID_DISPARITY_F_THRESH;
		i32 chkDirections[dirs][2] = { {1,1},{1,-1},{-1,1},{-1,-1},{0,1},{0,-1},{1,0},{-1,0} };
		u32* processItems[2][2] = { {occuList,occuLen},{misList,misLen} };
		u32* validNeighbours = allocate_mem(u32, dirs);
		i32 validNeighboursLen = 0;
		for (i32 i = 0; i < (i32)imageWidth; i++) {
			for (i32 j = 0; j < (i32)imageHeight; j++) {
				get_pixel(outMap, i, j, imageWidth, imageHeight) = get_pixel(disparityMap, i, j, imageWidth, imageHeight);
			}
		}
		for (i32 T = 0; T < 3; T++) {
			if (T < 2) {
				for (i32 i = 0; i < (i32)*processItems[T][1]; i++) {
					validNeighboursLen = 0;
					u32 idx = processItems[T][0][i];
					u32 tx = idx2xcoord(idx, imageWidth, imageHeight);
					u32 ty = idx2ycoord(idx, imageWidth, imageHeight);
					for (i32 j = 0; j < dirs; j++) {
						for (i32 dx = tx, dy = ty; (dx < (i32)imageWidth && dx >= 0) && (dy < (i32)imageHeight && dy >= 0); dx += chkDirections[j][0], dy += chkDirections[j][1]) {
							if (get_pixel(outMap, dx, dy, imageWidth, imageHeight) > invIdx) {
								validNeighbours[validNeighboursLen++] = coord2idx(dx, dy, imageWidth, imageHeight);
								break;
							}
						}
					}
					std::sort(validNeighbours, validNeighbours + validNeighboursLen);
					if (validNeighboursLen != 0) {
						//occlusion: Replace with second one
						if (T == 0) {
							if (validNeighboursLen == 1) {
								outMap[idx] = outMap[validNeighbours[0]];
							}
							else {
								outMap[idx] = outMap[validNeighbours[1]];
							}
						}
						//mismatch: Replace with the median
						else {
							outMap[idx] = outMap[validNeighbours[validNeighboursLen / 2]];
						}
					}
					
				}
			}
			else {
				//Process the remaining
				for (i32 tx = 0; tx < (i32)imageWidth; tx++) {
					for (i32 ty = 0; ty < (i32)imageHeight; ty++) {
						//Cur pos is (tx,ty)
						validNeighboursLen = 0;
						if (get_pixel(outMap, tx, ty, imageWidth, imageHeight) < invIdx) {
							//std::cout << "Filling" << tx << "," << ty << std::endl;
							for (i32 j = 0; j < dirs; j++) {
								for (i32 dx = tx, dy = ty; (dx < (i32)imageWidth && dx >= 0) && (dy < (i32)imageHeight && dy >= 0); dx += chkDirections[j][0], dy += chkDirections[j][1]) {
									if (get_pixel(outMap, dx, dy, imageWidth, imageHeight) > invIdx) {
										validNeighbours[validNeighboursLen++] = coord2idx(dx, dy, imageWidth, imageHeight);
										break;
									}
								}
							}
							std::sort(validNeighbours, validNeighbours + validNeighboursLen);
							if (validNeighboursLen != 0) {
								get_pixel(outMap, tx, ty, imageWidth, imageHeight) = outMap[validNeighbours[validNeighboursLen / 2]];
								
							}
							else {
								std::cout << "Warning:" << tx << "," << ty << std::endl;
							}
						}
						if (get_pixel(outMap, tx, ty, imageWidth, imageHeight) < invIdx) {
							std::cout << "Warning:" << tx << "," << ty << std::endl;
						}
					}
				}
			}
		}
	}
}