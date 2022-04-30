#include "../../../include/Common/cmTypeDefs.h"

namespace StereoMapping{
    class CostCalculator{
    public:
        virtual u32 smCostCalculate(u8* leftImage, u8* rightImage,u32 imageWidth,u32 imageHeight, u32 disparityRange,u8* &costOutput) = 0;
    };
};