#pragma once
#include "../../../include/Common/cmTypeDefs.h"
#include "../../../src/StereoMapping/CostCalculator/smCostCalculator.cpp"

namespace StereoMapping{
    class CostCalculator{
    public:
        virtual u32 smCostCalculate(u8* leftImage, u8* rightImage,u32 imageWidth,u32 imageHeight, u32 disparityRange,u8* costOutput) = 0;
        template<class T,class S> void smDisparityEstimate(T* costMatrix,S* outputMatrix,u32 imageWidth,u32 imageHeight, u32 disparityRange);
    };

    template<class T,class S> void CostCalculator::smDisparityEstimate(T* costMatrix,S* outputMatrix,u32 imageWidth,u32 imageHeight, u32 disparityRange){
        // Winner eats all
        for(u32 i=0;i<imageWidth;i++){
            for(u32 j=0;j<imageHeight;j++){
                i32 minDisparity = I32_MAX;
                i32 minDisparityIndex = 0;
                for(u32 k=0;k<disparityRange;k++){
                    if(get_pixel3(costMatrix,i,j,k,imageWidth,imageHeight,disparityRange)<(u32)minDisparity){
                        minDisparity = get_pixel3(costMatrix,i,j,k,imageWidth,imageHeight,disparityRange);
                        minDisparityIndex = k;
                    }
                }
                get_pixel(outputMatrix,i,j,imageWidth,imageHeight) = minDisparityIndex;
            }
        }
    }
};

