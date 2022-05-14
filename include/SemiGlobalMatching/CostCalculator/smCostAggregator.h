#pragma once
#include "../../Common/cmTypeDefs.h"

namespace SemiGlobalMatching{
    class CostAggregator{
    public:
        virtual void smCostAggregate(u8* imageData,u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix) = 0;
    };
}