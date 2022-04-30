#include "../../../include/StereoMapping/CostCalculator/smCostCalculator.h"

namespace StereoMapping {
    void CostCalculator::smGetAnotherCost(u8* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u8* costOutput) {
        //Left Cost (i,j,k) => Right(i+k,j,k)
        for (i32 i = 0; i < imageWidth; i++) {
            for (i32 j = 0; j < imageHeight; j++) {
                for (i32 k = 0; k < disparityRange; k++) {
                    if (i - k >= 0) {
                        get_pixel3(costOutput, i, j, k, imageWidth, imageHeight, disparityRange) = get_pixel3(costMatrix, i - k, j, k, imageWidth, imageHeight, disparityRange);
                    }
                    else {
                        get_pixel3(costOutput, i, j, k, imageWidth, imageHeight, disparityRange) = U8_MAX;
                    }
                }
            }
        }
    }
}
