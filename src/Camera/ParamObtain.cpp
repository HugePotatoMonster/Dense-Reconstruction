#include "../../include/Camera/ParamObtain.h"

namespace Camera{
    Matrix<float,3,3> ParamObtain::getIntrinsic(){
        Matrix<float,3,3> intrinsic = Matrix<float,3,3>::Zero();
        intrinsic(0,0) = 481.20;
        intrinsic(1,1) = 480.00;
        intrinsic(2,2) = 1;
        intrinsic(0,2) = 319.50;
        intrinsic(1,2) = 239.50;
        return intrinsic;
    };

    Matrix<float,3,3> ParamObtain::getIntrinsic(int sampleNo,int imgNo){
        Matrix<float,3,3> intrinsic = Matrix<float,3,3>::Zero();
        intrinsic(0,0) = 481.20;
        intrinsic(1,1) = 480.00;
        intrinsic(2,2) = 1;
        intrinsic(0,2) = 319.50;
        intrinsic(1,2) = 239.50;
        return intrinsic;
    };

    Matrix<float,3,4> ParamObtain::getExtrinsic(int sampleNo,int imgNo){
        return this->reader.readinPose(sampleNo,imgNo);
    };
};