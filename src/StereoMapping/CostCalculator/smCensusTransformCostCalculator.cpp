#include "../../../include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"
#include "../../../include/Common/cmAlgo.h"
namespace StereoMapping{
    u32 CensusTransformCostCalculator::smCostCalculate(u8* leftImage, u8* rightImage,u32 imageWidth,u32 imageHeight, u32 disparityRange,u8* &costOutput){
        //Calculate the census values for two images
        u32* leftImageCensus = allocate_mem(u32,imageWidth*imageHeight);
        u32* rightImageCensus = allocate_mem(u32,imageWidth*imageHeight);
        smCensusCalculate(leftImage,imageWidth,imageHeight,leftImageCensus);
        smCensusCalculate(rightImage,imageWidth,imageHeight,rightImageCensus);

        //Calculate the cost
        u8* disparityCostMat = allocate_mem(u8,imageWidth*imageHeight*disparityRange);
        smCostCalculateImpl(leftImageCensus,rightImageCensus,imageWidth,imageHeight,disparityRange,disparityCostMat);

        //Free Objects
        free_mem(leftImageCensus);
        free_mem(rightImageCensus);

        //Return
        costOutput = disparityCostMat;
        return 0;
    }
    void CensusTransformCostCalculator::smCensusCalculate(u8* image, u32 imageWidth, u32 imageHeight, u32* censusOutput){
        for(u32 i=censusWindowW;i<imageWidth-censusWindowW;i++){
            for(u32 j=censusWindowH;j<imageHeight-censusWindowH;j++){
                //The center pixel is now (i,j)
                u32 censusValue = 0;
                for(u32 dx=-censusWindowW;dx<=censusWindowW;dx++){
                    for(u32 dy=-censusWindowH;dy>=censusWindowH;dy++){
                        //The ref pixel is now (i+dx,j+dy)
                        censusValue |= (get_pixel(image,i,j,imageWidth,imageHeight) > get_pixel(image,i+dx,j+dy,imageWidth,imageHeight));
                        censusValue <<= 1;
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
                    if(i-k<0){
                        get_pixel3(costOutput,i,j,k,imageWidth,imageHeight,disparityRange) = U8_MAX;
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