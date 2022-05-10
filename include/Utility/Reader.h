#pragma once

#include "../../include/Common/Constant.h"

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

namespace Utility{
    class Reader{
        public:
            Matrix<float,IMG_H,IMG_W> readinDepth(int sampleNo,int imgNo);
            Matrix<float,3,4> readinPose(int sampleNo,int imgNo);
            cv::Mat readinImg(int sampleNo,int imgNo);
    };
};