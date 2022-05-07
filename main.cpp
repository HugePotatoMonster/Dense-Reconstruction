#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "./include/Common/cmAlgo.h"
#include "./include/StereoMapping/CostCalculator/smCostCalculator.h"
#include "./include/StereoMapping/CostCalculator/smCensusTransformCostCalculator.h"
#include "./include/StereoMapping/CostCalculator/smFourPathCostAggregator.h"
#include "./include/StereoMapping/CostCalculator/smEightPathCostAggregator.h"
#include "./include/StereoMapping/CostOptimizer/smCostOptimizer.h"
#include "./include/StereoMapping/Helper/smCostHelper.h"
#include "./include/StereoMapping/DepthEstimator/smDepthConverter.h"
#include "./include/DenseReconstruction/Voxel/drPlainVoxelStore.h"
#include "./include/DenseReconstruction/TSDF/drTSDF.h"

#define CE_TYPE u32

using namespace std;
using namespace cv;

int main() {
	cout << "Starting" << endl;
	//Create Objects
	StereoMapping::CostHelper helper = StereoMapping::CostHelper();
	StereoMapping::CostOptimizer optimizer = StereoMapping::CostOptimizer();
	StereoMapping::DepthConverter depthConverter = StereoMapping::DepthConverter();
	DenseReconstruction::TruncatedSDF tsdfCalc = DenseReconstruction::TruncatedSDF();

	//Load Images
	cout << "Loading Image" << endl;
	cv::Mat imageLeft = cv::imread("C:/WR/Sayu/samples/vl.png", 0);
	cv::Mat imageRight = cv::imread("C:/WR/Sayu/samples/vr.png", 0);
	/*
	cv::Mat imageLeftFlip = imageLeft.clone();
	cv::Mat imageRightFlip = imageRight.clone();
	cv::flip(imageLeft, imageLeftFlip, 1);
	cv::flip(imageRight, imageRightFlip, 1);*/

	u32 imageWidth = imageLeft.size[1];
	u32 imageHeight = imageLeft.size[0];
	u32 disparityRange = 64;
	cout << "IW=" << imageWidth << ", IH=" << imageHeight << endl;

	//Disparity Estimate
	cout << "Estimating Disparity" << endl;
	f64* leftDisparityMap = allocate_mem(f64, imageWidth * imageHeight);
	f64* leftDisparityMapS = allocate_mem(f64, imageWidth * imageHeight);
	u32* occlusionList = allocate_mem(u32, imageWidth * imageHeight);
	u32* mismatchList = allocate_mem(u32, imageWidth * imageHeight);
	u32 occlusionListLen = 0, mismatchListLen = 0;
	helper.calculateCostInternalF(imageLeft.data, imageRight.data, imageWidth, imageHeight, disparityRange, leftDisparityMap, leftDisparityMapS, occlusionList, &occlusionListLen, mismatchList, &mismatchListLen);

	//Connected Block Check
	cout << "Connected Block Check" << endl;
	f64* leftDisparityMapCbc = allocate_mem(f64, imageWidth * imageHeight);
	optimizer.smConnectedBlockFiltering(leftDisparityMap, leftDisparityMapCbc, imageWidth, imageHeight, 1.1, 20);

	
	//Disparity Fill
	cout << "Disparity Fill" << endl;
	f64* leftDisparityMapFl = allocate_mem(f64, imageWidth * imageHeight);
	optimizer.smDisparityFill(leftDisparityMapCbc, leftDisparityMapFl, imageWidth, imageHeight, occlusionList, &occlusionListLen, mismatchList, &mismatchListLen);

	//Median Filter
	cout << "Median Filter" << endl;
	f64* leftDisparityMapMf = allocate_mem(f64, imageWidth * imageHeight);
	optimizer.smMedianFilter(leftDisparityMapFl, leftDisparityMapMf, imageWidth, imageHeight, 5);


	//=========== End of Disparity Estimation ==================
	cout << "Depth Estimation" << endl;
	f64* depthMap = allocate_mem(f64, imageWidth * imageHeight);
	depthConverter.smIdealBinocularDisparityToDepth(leftDisparityMapMf, depthMap, imageWidth, imageHeight, 20.0, 60.0);


	//Discretization
	cout << "Discretization" << endl;
	u32* depthMapTrunc = allocate_mem(u32, imageWidth * imageHeight);
	u32 depthMapTruncMax = 0;
	depthConverter.smDepthDiscretization(depthMap, depthMapTrunc, &depthMapTruncMax, imageWidth, imageHeight);

	//=========== End of Depth Calculation ==================

	cout << "Creating Voxels" << endl;
	DenseReconstruction::VoxelStore* voxelStore = new DenseReconstruction::PlainVoxelStore();


	//Save PPM
	cout << "Saving PPM" << endl;
	Common::Algorithm::cmSaveAsPPM32("C:/WR/Sayu/samples/vs1-cb-3a.ppm", depthMapTrunc, imageWidth, imageHeight, depthMapTruncMax);

	return 0;
}