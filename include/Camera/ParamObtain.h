#pragma once

#include "../Common/Type.h"

#include <iostream>

// #include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
// using namespace Eigen;

namespace ParamHelper{
    class ParamObtain{
        public:
            // 3*3
            cv::Mat getIntrinsic();
            // 3*3
            cv::Mat getIntrinsic(int sampleNo,int imgNo);
            // 4*4
            cv::Mat getExtrinsic(int sampleNo,int imgNo);

        // TEST

            // 3*3
            cv::Mat getIntrinsic_test();
    };
};