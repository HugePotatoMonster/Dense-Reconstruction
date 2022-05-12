#pragma once

#include "../Common/Constant.h"
#include "../Utility/Log.h"

#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;

namespace Utility{

    cv::Mat rigidTransform(cv::Mat xyzs,cv::Mat transform){
        // xyzs: n*3
        // transform: 4*4
        // cout << "xyzs shape: " << xyzs.size() << endl;

        cv::Mat xyz_;
        cv::copyMakeBorder(xyzs,xyz_,0,0,0,1,CV_HAL_BORDER_CONSTANT,1);
        // cout << "xyz_ shape: " << xyz_.size() << endl;
        // cout << "transform shape: " << transform.size() << endl;

        // xyz_: 4*n
        xyz_ = (transform*xyz_.t()).t();
        // cout << "xyz_ shape: " << xyz_.size() << endl;

        // 3*5
        return xyz_(cv::Rect(0,0,3,xyz_.size[0]));
    };

    cv::Mat getFrustum(cv::Mat depth,cv::Mat intr,cv::Mat extr){
        // depth: img_H*img_W
        // intr: 3*3
        // extr 4*4
        double maxDepth;
        cv::minMaxIdx(depth, nullptr, &maxDepth);

        double fx = intr.at<double>(0,0);
        double fy = intr.at<double>(1,1);
        double x0 = intr.at<double>(0,2);
        double y0 = intr.at<double>(1,2);

        cv::Mat frustPts = (cv::Mat_<double>(3,5) << 
            -x0,-x0,-x0,IMG_W-x0,IMG_W-x0,
            -y0,-y0,IMG_H-y0,-y0,IMG_H-y0,
            0,1,1,1,1);

        // Utility::logMat(frustPts,"frustPts");

        cv::Mat multMat = (cv::Mat_<double>(3,5) << 
            0,maxDepth/fx,maxDepth/fx,maxDepth/fx,maxDepth/fx,
            0,maxDepth/fy,maxDepth/fy,maxDepth/fy,maxDepth/fy,
            0,maxDepth,maxDepth,maxDepth,maxDepth);

        // Utility::logMat(multMat,"multMat");

        cv::multiply(frustPts,multMat,frustPts);

        // Utility::logMat(frustPts,"frustPts");

        // frustPts: 3*5

        return rigidTransform(frustPts.t(), extr).t();
    }

};