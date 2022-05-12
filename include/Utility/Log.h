#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

using namespace std;

namespace Utility{
    void logMat(cv::Mat mat, string name, bool showType=true, bool showShape=true, bool showVal=true){
        if (showType){
            cout << name << " type:" << mat.type() << endl;
        }
        if (showShape){
            cout << name << " size:" << mat.size() << endl;
        }
        if (showVal){
            cout << name << " value:" << endl << mat << endl;
        }
    };
};