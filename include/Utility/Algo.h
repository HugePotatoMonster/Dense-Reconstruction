#pragma once

#include "../Common/Constant.h"

#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;

namespace Utility{
    class Algo{
        public:
            static cv::Mat rigidTransform(cv::Mat xyzs,cv::Mat transform);
            static cv::Mat getFrustum(cv::Mat depth,cv::Mat intr,cv::Mat extr);
            static cv::Mat vox2world(double* volOrigin, int** coords, int coordNum, double voxSize);
            static void cam2pix(cv::Mat camPts, cv::Mat intr, int coordNum, int* pix_x, int* pix_y);
    };

};