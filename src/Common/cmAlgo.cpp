#include "../../include/Common/cmAlgo.h"

namespace Common{
    u32 Algorithm::cmBitCount(u32 x){
        x = (x & 0x55555555) + ((x >> 1) & 0x55555555);
        x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
        x = (x & 0x0F0F0F0F) + ((x >> 4) & 0x0F0F0F0F);
        return (x * 0x01010101) >> 24;;
    }
    u32 Algorithm::cmHammingDistance(u32 x,u32 y){
        return Algorithm::cmBitCount(x^y);
    }
    u32 Algorithm::cmSaveAsPPM(std::string file_path, u8* imageData,u32 imageWidth,u32 imageHeight,u8 maxValue){
        std::ofstream imagePPMOutput(file_path);
        imagePPMOutput<<"P3"<<std::endl<<imageWidth<<" "<<imageHeight<<std::endl<<maxValue<<std::endl;
        for(u32 j=0;j<imageHeight;j++){ //Rows
            for(u32 i=0;i<imageWidth;i++){ //Cols
                imagePPMOutput<<(i32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(i32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(i32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" ";
            }
            imagePPMOutput<<std::endl;
        }
        imagePPMOutput.close();
        return 0;
    }
    u32 Algorithm::cmSaveAsPPM32(std::string file_path, u32* imageData,u32 imageWidth,u32 imageHeight,u32 maxValue){
        std::ofstream imagePPMOutput(file_path);
        imagePPMOutput<<"P3"<<std::endl<<imageWidth<<" "<<imageHeight<<std::endl<<maxValue<<std::endl;
        for(u32 j=0;j<imageHeight;j++){ //Rows
            for(u32 i=0;i<imageWidth;i++){ //Cols
                imagePPMOutput<<(u32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(u32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(u32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" ";
            }
            imagePPMOutput<<std::endl;
        }
        imagePPMOutput.close();
        return 0;
    }
}