#include "../../include/Utility/Log.h"

using namespace std;

namespace Utility{
    void Log::logMat(cv::Mat mat, string name, bool showType, bool showShape, bool showVal){
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