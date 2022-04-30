#include "../../../include/StereoMapping/CostCalculator/smFourPathCostAggregator.h"
namespace StereoMapping{
    void FourPathCostAggregator::smCostAggregate(u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix){
        
    }
    void FourPathCostAggregator::smCostAggregateLR(u8* costMatrix,u32 imageWidth,u32 imageHeight,u32 disparityRange, u32* refinedMatrix, u8 direction){
        //Optimization problem: f[i,j,k] = C[i,j,k] + min(f[ix,jx,k],f[ix,jx,k+1]+p1,f[ix,jx,k-1]+p1,f[ix,jx,t]+p2)-min(f[ix,jx,t])
        //Array scrolling for "f" to reduce memory consumption
        u32 lastMin = U32_MAX;
        u32* optCost = allocate_mem(u32,disparityRange*2);

        for(u32 j=0;j<imageHeight;j++){
            //Current pixel is (?,j)
            i32 startCoord = direction?imageWidth-1:0;
            i32 stopCoord = direction?-1:imageWidth;
            i32 deltaCoord = direction?-1:1;
            for(i32 k=0;k<disparityRange*2;k++){
                optCost[k] = U32_MAX;
            }
            //Init Cond for Dynamic Programming
            for(i32 k=0;k<disparityRange;k++){
                lastMin = Min(lastMin,get_pixel(optCost,k,startCoord&1,disparityRange,2) = get_pixel3(costMatrix,startCoord,j,k,imageWidth,imageHeight,disparityRange));
                get_pixel3(refinedMatrix,startCoord,j,k,imageWidth,imageHeight,disparityRange) = get_pixel3(costMatrix,startCoord,j,k,imageWidth,imageHeight,disparityRange);
            }
            //Status Updating
            for(i32 k=startCoord+deltaCoord;k!=stopCoord;k+=deltaCoord){
                //Updating f[k,j,d] from f[k',j,d']
                i32 newMin = U32_MAX;
                for(i32 d=0;d<disparityRange;d++){
                    get_pixel(optCost,d,k&1,disparityRange,2) = get_pixel3(costMatrix,k,j,d,imageWidth,imageHeight,disparityRange);
                    i32 addedValue= get_pixel(optCost,d-1,(k+1)&1,disparityRange,2);
                    if(d>0){
                        addedValue = Min(addedValue,get_pixel(optCost,d-1,(k+1)&1,disparityRange,2)+p1);
                    }
                    if(d<disparityRange-1){
                        addedValue = Min(addedValue,get_pixel(optCost,d+1,(k+1)&1,disparityRange,2)+p1);
                    }
                    addedValue = Min(addedValue,lastMin+p2);
                    get_pixel(optCost,d,k&1,disparityRange,2) += (addedValue-lastMin);
                    newMin = Min(newMin,get_pixel(optCost,d,k&1,disparityRange,2));
                    get_pixel3(refinedMatrix,k,j,d,imageWidth,imageHeight,disparityRange) = get_pixel(optCost,d,k&1,disparityRange,2);
                }
                lastMin = newMin;
            }
        }

    }

}