#pragma once
#include "smCostCalculator.h"

namespace SemiGlobalMatching{
    class CensusTransformCostCalculator:public CostCalculator{
    private:
        u32 censusWindowH = 2;
        u32 censusWindowW = 2;
    public:
        u32 smCostCalculate(u8* leftImage, u8* rightImage,u32 imageWidth,u32 imageHeight, u32 disparityRange, u8* costOutput) override;
        void smCensusCalculate(u8* leftImage, u32 imageWidth, u32 imageHeight, u32* censusOutput);
        void smCostCalculateImpl(u32* leftCensus, u32* rightCensus, u32 imageWidth,u32 imageHeight, u32 disparityRange, u8* costOutput);
    };
}