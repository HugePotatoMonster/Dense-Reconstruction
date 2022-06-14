#pragma once
#include "../Common/cmTypeDefs.h"

namespace Test{
    class Test{
    public:
        static void stereoRectify();
        static void epipolarLineProjection();
        static void binocularSGM();
        static void sgmMarchingCubeSurface();
        static void monocularMotionSGM();
        static void monocularMotionSGMDepth();
        static void monocularMotionSGMDepthFinal();
        static void cudaObjCreation();
        static void stdMapTest();
        static void oglTest();
        static void testj();
        static double* use_filter(double* depthMap_left, double* depthMap_right, uint64_t imageWidth, uint64_t imageHeight, const cv::Mat left, const cv::Mat right);
        static void transfer_depth(double* d_i, double* d, int iter, int flag, uint64_t imageWH);
    };
}