#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "./include/Common/cmAlgo.h"
#include "./include/StereoMapping/CostCalculator/smCostCalculator.h"
#include "./include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"
#include "./include/StereoMapping/CostCalculator/smFourPathCostAggregator.h"

using namespace std;
using namespace cv;

int main(){
    cout<<"Starting"<<endl;
    //Create Objects
    StereoMapping::CostCalculator* costEstimator = new StereoMapping::CensusTransformCostCalculator();
    StereoMapping::CostAggregator* costAggregator = new StereoMapping::FourPathCostAggregator();

    //Load Images
    cv::Mat imageLeft = cv::imread("C:/WR/Sayu/samples/vl.png",0);
    cv::Mat imageRight = cv::imread("C:/WR/Sayu/samples/vr.png",0);
    u32 imageWidth = imageLeft.size[1];
    u32 imageHeight = imageLeft.size[0];
    u32 disparityRange = 128;

    cout<<"Loaded"<<endl;
    //Estimate Cost
    u8* costMatrix = allocate_mem(u8,imageWidth*imageHeight*disparityRange);
    costEstimator->smCostCalculate(imageLeft.data,imageRight.data,imageWidth,imageHeight,disparityRange,costMatrix);
    cout<<"Cost Estimate"<<endl;

    //Save PPM
    u32* disparityMapWR = allocate_mem(u32,imageWidth*imageHeight);
    costEstimator->smDisparityEstimate<u8,u32>(costMatrix,disparityMapWR,imageWidth,imageHeight,disparityRange);
    cout<<"Disparity Estimate Without Aggregation"<<endl;
    Common::Algorithm::cmSaveAsPPM32("C:/WR/Sayu/samples/vs1-wa.ppm",disparityMapWR,imageWidth,imageHeight,disparityRange);
    cout<<"PPM Saved"<<endl;

    //Cost Aggregation
    u32* refinedCostMatrix = allocate_mem(u32,imageWidth*imageHeight*disparityRange);
    set_zero(refinedCostMatrix,sizeof(u32)*imageWidth*imageHeight*disparityRange);
    costAggregator->smCostAggregate(imageLeft.data,costMatrix,imageWidth,imageHeight,disparityRange,refinedCostMatrix);
    cout<<"Cost Aggregation"<<endl;

    //Winner Takes All
    u32* disparityMap = allocate_mem(u32,imageWidth*imageHeight);
    costEstimator->smDisparityEstimateSubpixelRefine<u32,u32>(refinedCostMatrix,disparityMap,imageWidth,imageHeight,disparityRange);
    cout<<"Disparity Estimate"<<endl;

    //Save PPM
    Common::Algorithm::cmSaveAsPPM32("C:/WR/Sayu/samples/vs1-cb.ppm",disparityMap,imageWidth,imageHeight,disparityRange);
    cout<<"PPM Saved"<<endl;
    return 0;
}