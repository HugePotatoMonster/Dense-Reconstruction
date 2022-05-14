#include "../../../include/SemiGlobalMatching/CostCalculator/smCostCalculator.h"

namespace SemiGlobalMatching {
    void CostCalculator::smGetAnotherCost(u32* costMatrix, u32 imageWidth, u32 imageHeight, u32 disparityRange, u32* costOutput) {
        //Left Cost (i,j,k) => Right(i+k,j,k)
        for (i32 i = 0; i < imageWidth; i++) {
            for (i32 j = 0; j < imageHeight; j++) {
                for (i32 k = 0; k < disparityRange; k++) {
                    if (i + k < imageWidth) {
                        get_pixel3(costOutput, i, j, k, imageWidth, imageHeight, disparityRange) = get_pixel3(costMatrix, i + k, j, k, imageWidth, imageHeight, disparityRange);
                    }
                    else {
                        get_pixel3(costOutput, i, j, k, imageWidth, imageHeight, disparityRange) = I32_MAX;
                    }
                }
            }
        }
    }
}
