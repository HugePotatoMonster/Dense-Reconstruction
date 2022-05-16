#pragma once
#include "../../../include/SemiGlobalMatching/CostCalculator/smCostAggregator.h"
namespace SemiGlobalMatching {
    class EightPathCostAggregatorCuda :public CostAggregator {
    public:
        u32 p1 = 10; //Penalty coefficient for cost difference equals to 1
        u32 p2 = 150; //Penalty coefficient for cost difference larger than 1
        i32 div = 4;
    public:
        void smCostAggregate(u8* imageData, u32* costMatrix, u32 imageWidth, u32 imageHeight, i32 minDisparity, u32 disparityRange, u32* refinedMatrix) override;
    };
}