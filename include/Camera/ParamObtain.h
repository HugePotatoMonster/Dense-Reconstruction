#pragma once

#include "../Common/Type.h"
#include "../../include/Utility/Reader.h"

#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace Camera{
    class ParamObtain{
        private:
            Utility::Reader reader;
        public:
            Matrix<float,3,3> getIntrinsic();
            Matrix<float,3,3> getIntrinsic(int sampleNo,int imgNo);
            Matrix<float,3,4> getExtrinsic(int sampleNo,int imgNo);
    };
};