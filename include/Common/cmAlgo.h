#pragma once
#include "../../include/Common/cmTypeDefs.h"

namespace Common{
    class Algorithm{
        public:
            inline static u32 cmBitCount(u32 x);
            inline static u32 cmHammingDistance(u32 x, u32 y);
    };
};