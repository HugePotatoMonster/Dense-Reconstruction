#include "../../include/Camera/ParamObtain.h"
#include "../../include/Utility/Reader.h"

namespace ParamHelper{
    cv::Mat ParamObtain::getIntrinsic(){
        // Matrix<double,3,3> intrinsic = Matrix<double,3,3>::Zero();
        cv::Mat intrinsic = cv::Mat::zeros(3,3,CV_64FC1);
        intrinsic.at<double>(0,0) = 481.20;
        intrinsic.at<double>(1,1) = 480.00;
        intrinsic.at<double>(2,2) = 1;
        intrinsic.at<double>(0,2) = 319.50;
        intrinsic.at<double>(1,2) = 239.50;
        return intrinsic;
    };

    cv::Mat ParamObtain::getIntrinsic(int sampleNo,int imgNo){
        cv::Mat intrinsic = cv::Mat::zeros(3,3,CV_64FC1);
        intrinsic.at<double>(0,0) = 481.20;
        intrinsic.at<double>(1,1) = 480.00;
        intrinsic.at<double>(2,2) = 1;
        intrinsic.at<double>(0,2) = 319.50;
        intrinsic.at<double>(1,2) = 239.50;
        return intrinsic;
    };

   cv::Mat ParamObtain::getExtrinsic(int sampleNo,int imgNo){
        return Utility::Reader::getInstance()->readinPose(sampleNo,imgNo);
    };

// TEST

    cv::Mat ParamObtain::getIntrinsic_test(){
        cv::Mat intrinsic = cv::Mat::zeros(3,3,CV_64FC1);
        intrinsic.at<double>(0,0) = 585;
        intrinsic.at<double>(1,1) = 585;
        intrinsic.at<double>(2,2) = 1;
        intrinsic.at<double>(0,2) = 320;
        intrinsic.at<double>(1,2) = 240;
        return intrinsic;
    };

};