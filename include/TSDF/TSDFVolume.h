#pragma once
// #include "../Utility/Log.h"

#include <opencv2/opencv.hpp>

namespace TSDF{
    class TSDFVolume{
        private:
            cv::Mat _bound;
            double _voxSize;
            double _volOrigin[3];
            double _volDim[3];

            double*** _tsdf;
            double*** _weight;
            double*** _color;

            int** _coords;
            int _coordNum;

        public:
            TSDFVolume(cv::Mat bound, double voxSiz=0.02);
    };
};