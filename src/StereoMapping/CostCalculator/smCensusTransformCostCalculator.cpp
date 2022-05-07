#include "../../../include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"
#include "../../../include/Common/cmAlgo.h"
#include <iostream>
namespace StereoMapping{
    u32 CensusTransformCostCalculator::smCostCalculate(u8* leftImage, u8* rightImage,u32 imageWidth,u32 imageHeight, u32 disparityRange,u8* costOutput){
        //Calculate the census values for two images
        u32* leftImageCensus = allocate_mem(u32,imageWidth*imageHeight);
        u32* rightImageCensus = allocate_mem(u32,imageWidth*imageHeight);
        smCensusCalculate(leftImage,imageWidth,imageHeight,leftImageCensus);
        smCensusCalculate(rightImage,imageWidth,imageHeight,rightImageCensus);
        //Calculate the cost
        smCostCalculateImpl(leftImageCensus,rightImageCensus,imageWidth,imageHeight,disparityRange,costOutput);
        //Free Objects
        free_mem(leftImageCensus);
        free_mem(rightImageCensus);

        return 0;
    }
    void CensusTransformCostCalculator::smCensusCalculate(u8* image, u32 imageWidth, u32 imageHeight, u32* censusOutput){
        for(u32 i=censusWindowW;i<imageWidth-censusWindowW;i++){
            for(u32 j=censusWindowH;j<imageHeight-censusWindowH;j++){
                //The center pixel is now (i,j)
                u32 censusValue = 0;
                for(i32 dx=-(i32)censusWindowW;dx<=(i32)censusWindowW;dx++){
                    for(i32 dy=-(i32)censusWindowH;dy<=(i32)censusWindowH;dy++){
                        //The ref pixel is now (i+dx,j+dy)
                        censusValue <<= 1;
                        censusValue |= (get_pixel(image,i,j,imageWidth,imageHeight) > get_pixel(image,(i32)(i)+dx,(i32)(j)+dy,imageWidth,imageHeight));
                        
                    }
                }
                get_pixel(censusOutput,i,j,imageWidth,imageHeight) = censusValue;
            }
        }
    }
    void CensusTransformCostCalculator::smCostCalculateImpl(u32* leftCensus, u32* rightCensus, u32 imageWidth,u32 imageHeight, u32 disparityRange, u8* costOutput){
        for(u32 i = 0;i<imageWidth;i++){
            for(u32 j=0;j<imageHeight;j++){
                //Left pixel is (i,j)
                for(u32 k=0;k<disparityRange;k++){
                    //Right pixel is (i-k,j)
                    if(i<k){
                        get_pixel3(costOutput,i,j,k,imageWidth,imageHeight,disparityRange) = I8_MAX;
                    }else{
                        get_pixel3(costOutput,i,j,k,imageWidth,imageHeight,disparityRange) = Common::Algorithm::cmHammingDistance(
                            get_pixel(leftCensus,i,j,imageWidth,imageHeight),
                            get_pixel(rightCensus,i-k,j,imageWidth,imageHeight)
                        );
                    }
                }
            }
        }
    }
}