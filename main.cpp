#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "./include/Common/cmAlgo.h"
#include "./include/StereoMapping/CostCalculator/smCostCalculator.h"
#include "./include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"

using namespace std;
using namespace cv;

int main(){
    cout<<"Starting"<<endl;
    //Create Objects
    StereoMapping::CostCalculator* costEstimator = new StereoMapping::CensusTransformCostCalculator();

    //Load Images
    cv::Mat imageLeft = cv::imread("C:/WR/Sayu/samples/vl.png",0);
    cv::Mat imageRight = cv::imread("C:/WR/Sayu/samples/vr.png",0);
    u32 imageWidth = imageLeft.size[1];
    u32 imageHeight = imageLeft.size[0];
    u32 disparityRange = 64;

    cout<<"Loaded"<<endl;
    //Estimate Cost
    u8* costMatrix = allocate_mem(u8,imageWidth*imageHeight*disparityRange);
    costEstimator->smCostCalculate(imageLeft.data,imageRight.data,imageWidth,imageHeight,disparityRange,costMatrix);
    cout<<"Cost Estimate - 1"<<endl;
    //Winner Takes All
    u8* disparityMap = allocate_mem(u8,imageWidth*imageHeight);
    costEstimator->smDisparityEstimate<u8,u8>(costMatrix,disparityMap,imageWidth,imageHeight,disparityRange);
    cout<<"Cost Estimate - 2"<<endl;
    //Save PPM
    Common::Algorithm::cmSaveAsPPM("C:/WR/Sayu/samples/vs1.ppm",disparityMap,imageWidth,imageHeight,disparityRange);
    cout<<"PPM Saved"<<endl;
    return 0;
}