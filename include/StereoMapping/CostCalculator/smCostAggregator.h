#include "../../Common/cmTypeDefs.h"

namespace StereoMapping{
    class CostAggregator{
    public:
        virtual void smCostAggregate(u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix) = 0;
    };
}