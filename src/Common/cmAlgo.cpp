#include "../../include/Common/cmAlgo.h"

namespace Common{
    u32 Algorithm::cmBitCount(u32 x){
        x = (x & 0x55555555) + ((x >> 1) & 0x55555555);
        x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
        x = (x & 0x0F0F0F0F) + ((x >> 4) & 0x0F0F0F0F);
        return ((x * 0x01010101) & ((1 << 32) - 1)) >> 24;
    }
    u32 Algorithm::cmHammingDistance(u32 x,u32 y){
        return Algorithm::cmBitCount(x^y);
    }
}