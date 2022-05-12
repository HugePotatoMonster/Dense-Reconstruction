#pragma once

#include "../../include/Common/Constant.h"

#include <iostream>
// #include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
// using namespace Eigen;

namespace Utility{
    class Reader{

        private:
            Reader(){};
            static Reader* reader;

        public:
            static Reader* getInstance();

            // img_H*img_W
            cv::Mat readinDepth(int sampleNo,int imgNo);
            // 4*4
            cv::Mat readinPose(int sampleNo,int imgNo);
            // img_H*img_W
            cv::Mat readinImg(int sampleNo,int imgNo);

        // TEST

            // img_H*img_W
            cv::Mat readinDepth_test(int imgNo);
            // img_H*img_W
            cv::Mat readinPose_test(int imgNo);
            // img_H*img_W
            cv::Mat readinImg_test(int imgNo);
    };

};