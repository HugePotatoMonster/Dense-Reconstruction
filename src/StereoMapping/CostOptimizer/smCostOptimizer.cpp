#include "../../../include/StereoMapping/CostOptimizer/smCostOptimizer.h"

namespace StereoMapping {
	void CostOptimizer::smInternalConsistencyCheckF(f64* leftDisparityMap, f64* rightDisparityMap, f64* outputDisparityMap, u32 imageWidth, u32 imageHeight, f64 consistencyThreshold, f64 invalidPlaceholder) {
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				f64 leftDisp = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				i32 rightDispEpi = (i32)(i - leftDisp + 0.5);
				f64 rightDisp = 0;
				if (rightDispEpi < imageWidth && rightDispEpi >= 0) {
					rightDisp = get_pixel(rightDisparityMap, rightDispEpi, j, imageWidth, imageHeight);
				}
				f64 criteria = Abs(leftDisp - rightDisp);
				if (criteria <= consistencyThreshold) {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				}
				else {
					get_pixel(outputDisparityMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
				}
			}
		}
	}
	void CostOptimizer::smDisparityMapDiscretization(f64* disparityMap, u32* outputMap, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32 invalidPlaceholder) {
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				get_pixel(outputMap, i, j, imageWidth, imageHeight) = get_pixel(disparityMap, i, j, imageWidth, imageHeight);
				if (get_pixel(disparityMap, i, j, imageWidth, imageHeight) > (f64)disparityRange) {
					get_pixel(outputMap, i, j, imageWidth, imageHeight) = invalidPlaceholder;
				}
			}
		}
	}
	void CostOptimizer::smExternalConsistencyCheckI(u32* leftDisparityMap, u32* rightDisparityMap, u32* outputDisparityMap, u32 imageWidth, u32 imageHeight, u32 consistencyThreshold, u32 invalidPlaceholder) {
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				u32 leftDisp = get_pixel(leftDisparityMap, i, j, imageWidth, imageHeight);
				u32 rightDisp = get_pixel_hf(rightDisparityMap, i, j, imageWidth, imageHeight);
				u32 criteria = Abs(leftDisp - rightDisp);
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
		u8* visState = allocate_mem(u8, imageWidth * imageHeight);
		u32* bfsQueue = allocate_mem(u32, imageWidth * imageHeight);
		i32 bfsQueueRear = 0;
		i32 bfsQueueFront = 0;
		f64 curPixel = 0;
		const i32 bfsQueueLen = imageWidth * imageHeight;
		set_zero(visState, sizeof(u8) * imageWidth * imageHeight);
		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
				get_pixel(outputMap, i, j, imageWidth, imageHeight) = get_pixel(disparityMap, i, j, imageWidth, imageHeight);
			}
		}

		for (i32 i = 0; i < imageWidth; i++) {
			for (i32 j = 0; j < imageHeight; j++) {
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
							if (Abs(get_pixel(disparityMap, tx - 1, ty, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx - 1, ty, imageWidth, imageHeight)) {
									get_pixel(visState, tx - 1, ty, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx - 1, ty, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen)
								}
							}
						}
						if (ty > 0 && tx >= 0 && tx < imageWidth && ty < imageHeight) {
							if (Abs(get_pixel(disparityMap, tx, ty - 1, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx, ty - 1, imageWidth, imageHeight)) {
									get_pixel(visState, tx, ty - 1, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx, ty - 1, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen)
								}
							}
						}
						if (tx < imageWidth - 1 && ty < imageHeight) {
							if (Abs(get_pixel(disparityMap, tx + 1, ty, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx + 1, ty, imageWidth, imageHeight)) {
									get_pixel(visState, tx + 1, ty, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx + 1, ty, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen)
								}
							}
						}

						if (tx < imageWidth && ty < imageHeight - 1) {
							if (Abs(get_pixel(disparityMap, tx, ty + 1, imageWidth, imageHeight) - curPixel) < discriminationThreshold) {
								if (!get_pixel(visState, tx, ty + 1, imageWidth, imageHeight)) {
									get_pixel(visState, tx, ty + 1, imageWidth, imageHeight) = 1;
									queue_push(bfsQueue, coord2idx(tx, ty + 1, imageWidth, imageHeight), bfsQueueFront, bfsQueueRear, bfsQueueLen)
								}
							}
						}
					}
					if (bfsQueueRear < retainThreshold) {
						for (i32 i = 0; i < bfsQueueRear; i++) {
							outputMap[bfsQueue[i]] = 0;
						}
					}
				}
			}
		}

		free_mem(visState);
		free_mem(bfsQueue);
	}
}