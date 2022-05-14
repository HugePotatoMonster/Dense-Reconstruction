#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;

namespace Utility{
    class Log{
        public:
            static void logMat(cv::Mat mat, string name, bool showType=true, bool showShape=true, bool showVal=true);
    };
};