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

    };
}