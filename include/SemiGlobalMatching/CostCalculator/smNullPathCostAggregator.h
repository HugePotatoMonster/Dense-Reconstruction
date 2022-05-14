#pragma once
#include "../../../include/SemiGlobalMatching/CostCalculator/smCostAggregator.h"
namespace SemiGlobalMatching {
    class NullPathCostAggregator :public CostAggregator {
    public:

    public:
        void smCostAggregate(u8* imageData, u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* refinedMatrix);
    };
}