#pragma once
#include "../../Common/cmTypeDefs.h"

//This file are declarations of CUDA functions for CUDA does not supports member function for global
namespace Parallel {
	namespace CUDA {
		namespace Test {
			cu_global void helloworld();
			void helloworldCall();
		}
		namespace SemiGlobalMatching {
			namespace CostAggregator {
				cu_global void cusmParallelCostAggregationLR(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR);
				cu_global void cusmParallelCostAggregationUD(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR);
				cu_global void cusmParallelCostAggregationND(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR);
				cu_global void cusmParallelCostAggregationPD(CU_ARG u8* imageData, CU_ARG u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, CU_ARG OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, CU_ARG u32* optCostR);
				void cusmParallelCostAggregationFourPathCaller(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, OUT_ARG u32* refinedMatrix, u8 direction,u8 threads, u32* optCost);
				void cusmParallelCostAggregationEightPathCaller(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, OUT_ARG u32* refinedMatrix, u8 direction, u8 threads, u32* optCost);
			}
		}
	}
}