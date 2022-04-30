#pragma once
#include "../../../include/Common/cmTypeDefs.h"
#include "../../../src/StereoMapping/CostCalculator/smCostCalculator.cpp"

namespace StereoMapping {
	class CostCalculator {
	public:
		virtual u32 smCostCalculate(u8* leftImage, u8* rightImage, u32 imageWidth, u32 imageHeight, u32 disparityRange, u8* costOutput) = 0;
		void smGetAnotherCost(u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u8* costOutput);
		template<class T, class S> void smDisparityEstimate(T* costMatrix, S* outputMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange);
		template<class T, class S> void smDisparityEstimateSubpixelRefine(T* costMatrix, S* outputMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange);
	};

	template<class T, class S> void CostCalculator::smDisparityEstimate(T* costMatrix, S* outputMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange) {
		// Winner takes all
		for (u32 i = 0; i < imageWidth; i++) {
			for (u32 j = 0; j < imageHeight; j++) {
				i32 minDisparity = I32_MAX;
				i32 minDisparityIndex = 0;
				for (u32 k = 0; k < disparityRange; k++) {
					if (get_pixel3(costMatrix, i, j, k, imageWidth, imageHeight, disparityRange) < (u32)minDisparity) {
						minDisparity = get_pixel3(costMatrix, i, j, k, imageWidth, imageHeight, disparityRange);
						minDisparityIndex = k;
					}
				}
				get_pixel(outputMatrix, i, j, imageWidth, imageHeight) = minDisparityIndex;
			}
		}
	}
	template<class T, class S> void CostCalculator::smDisparityEstimateSubpixelRefine(T* costMatrix, S* outputMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange) {
		// Winner takes all
		// 
		// Subpixel Estimate:
		// 
		// Three points on the curve f(x)=ax^2+bx+c (d-1,A),(d,C),(d+1,B)
		// a(d^2-2d+1)+b(d-1)+c=A
		// ad^2+bd+c=C
		// a(d^2+2d+1)+b(d+1)+c=B
		// Then
		// -2ad-b+a=A-C
		// 2ad+b+a=B-C => a=(A-C)+(B-C)/2 => b=(B-A)/2-(A+B-2C)d
		// Best = -b/(A+B-2C) = d+(B-A)/2(A+B-2C)
		//
		for (u32 i = 0; i < imageWidth; i++) {
			for (u32 j = 0; j < imageHeight; j++) {
				i32 minDisparity = I32_MAX;
				i32 minDisparityIndex = 0;
				for (u32 k = 0; k < disparityRange; k++) {
					if (get_pixel3(costMatrix, i, j, k, imageWidth, imageHeight, disparityRange) < (u32)minDisparity) {
						minDisparity = get_pixel3(costMatrix, i, j, k, imageWidth, imageHeight, disparityRange);
						minDisparityIndex = k;
					}
				}
				if (minDisparityIndex != 0 && minDisparityIndex != disparityRange - 1) {
					f64 dA = get_pixel3(costMatrix, i, j, minDisparityIndex - 1, imageWidth, imageHeight, disparityRange);
					f64 dB = get_pixel3(costMatrix, i, j, minDisparityIndex + 1, imageWidth, imageHeight, disparityRange);
					f64 dC = get_pixel3(costMatrix, i, j, minDisparityIndex, imageWidth, imageHeight, disparityRange);
					get_pixel(outputMatrix, i, j, imageWidth, imageHeight) = minDisparityIndex + (dB - dA) / (dA + dB - 2.0 * dC) / 2.0;
				}
				else {
					get_pixel(outputMatrix, i, j, imageWidth, imageHeight) = minDisparityIndex;
				}
			}
		}
	}
};

