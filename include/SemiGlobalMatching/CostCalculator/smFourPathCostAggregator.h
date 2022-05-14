#pragma once
#include "../../../include/SemiGlobalMatching/CostCalculator/smCostAggregator.h"
namespace SemiGlobalMatching {
    class FourPathCostAggregator:public CostAggregator{
    public:
        u32 p1 = 10; //Penalty coefficient for cost difference equals to 1
        u32 p2 = 150; //Penalty coefficient for cost difference larger than 1
        i32 div = 4;
    public:
        void smCostAggregate(u8* imageData,u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix) override;
        virtual void smCostAggregateLR(u8* imageData,u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix,u8 direction);
        virtual void smCostAggregateUD(u8* imageData,u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix,u8 direction);
    };
}