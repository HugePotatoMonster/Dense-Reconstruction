#include <iostream>
#include <opencv2/opencv.hpp>

#include "./include/Test.h"
#include "./include/Common/cmAlgo.h"

using namespace std;
using namespace cv;

int main(){
    Test* a = new Test();
    
    cv::Mat hello = cv::imread("C:\\WR\\Sayu\\origin.jpeg",0);
    cout<<"Goodbye World"<<a->hello()<<","<<(Common::Algorithm::cmBitCount(0x7777))<<endl;
    cv::imshow("Goodbye", hello);
    cv::waitKey(0);
    return 0;
}