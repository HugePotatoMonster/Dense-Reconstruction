#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include "../../include/Common/cmTypeDefs.h"

namespace Common{
    class Algorithm{
        public:
            static u32 cmBitCount(u32 x);
            static u32 cmHammingDistance(u32 x, u32 y);
            static u32 cmSaveAsPPM(std::string file_path, u8* imageData,u32 imageWidth,u32 imageHeight,u8 maxValue=255);
            static u32 cmSaveAsPPM32(std::string file_path, u32* imageData,u32 imageWidth,u32 imageHeight,u32 maxValue=255);
    };
};