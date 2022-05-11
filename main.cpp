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
#include "./include/Common/Utility/cmVisExt.h"
#include "./include/StereoMapping/Helper/smDisparityHelper.h"

#include "./include/Misc/msAuxiliaryUtils.h"
#define CE_TYPE u32
#define SGM_ONLY true

using namespace std;
using namespace cv;

int main() {
	cout << "Starting" << endl;
	

	//Create Objects
	StereoMapping::DisparityHelper* disparityHelper = new StereoMapping::DisparityHelper();
	StereoMapping::CostOptimizer* discretizator = new StereoMapping::CostOptimizer();
	StereoMapping::DepthConverter depthConverter = StereoMapping::DepthConverter();
	DenseReconstruction::TruncatedSDF tsdfCalc = DenseReconstruction::TruncatedSDF();
	Common::Util::VisualizationExt visExt = Common::Util::VisualizationExt();
	//Test CamInt
	Common::Camera::MonocularCameraIntrinsic camIntb;
	Common::Camera::MonocularCameraExtrinsic camExtb;
	Misc::AuxiliaryUtils::msParseIntrinsic("E:\\60fps_GT_archieve\\scene_01_0209.txt",&camIntb);
	cout<<"Fx="<<camIntb.fx<<",Fy="<<camIntb.fy<<endl;
	cout<<"Cx="<<camIntb.cx<<",Cy="<<camIntb.cy<<endl;
	Misc::AuxiliaryUtils::msParseExtrinsic("E:\\60fps_GT_archieve\\scene_01_0209.txt",&camExtb);
	for(i32 i=0;i<3;i++){
		cout<<"T="<<camExtb.t[i]<<" ";
	}
	cout<<endl;
	for(i32 i=0;i<3;i++){
		for(i32 j=0;j<3;j++){
			cout<<"R="<<camExtb.r[i][j];
		}
		cout<<endl;
	}
	//Load Images
	cout << "Loading Image" << endl;
	cv::Mat imageLeft = cv::imread("C:/WR/Dense-Reconstruction/samples/vl.png", 0);
	cv::Mat imageRight = cv::imread("C:/WR/Dense-Reconstruction/samples/vr.png", 0);


	u32 imageWidth = imageLeft.size[1];
	u32 imageHeight = imageLeft.size[0];
	u32 disparityRange = 64;
	cout << "IW=" << imageWidth << ", IH=" << imageHeight << endl;

	//Disparity Estimate
	cout << "Estimating Disparity" << endl;
	f64* leftDisparityMap = allocate_mem(f64, (usize)imageWidth * imageHeight);
	disparityHelper->smIdealBinocularDisparity(imageLeft.data, imageRight.data, imageWidth, imageHeight, disparityRange, leftDisparityMap);

	cout << "Discretization" << endl;
	u32* leftDisparityMapMfds = allocate_mem(u32, (usize)imageWidth * imageHeight);
	discretizator->smDisparityMapDiscretization(leftDisparityMap, leftDisparityMapMfds, imageWidth, imageHeight, disparityRange, 0);

	cout << "Saving PPM" << endl;
	Common::Algorithm::cmSaveAsPPM32("C:/WR/Dense-Reconstruction/samples/vs1-cb-3da.ppm", leftDisparityMapMfds, imageWidth, imageHeight, disparityRange);

	//=========== End of Disparity Estimation ==================

	if (!SGM_ONLY) {	
		cout << "Depth Estimation" << endl;
		f64* depthMap = allocate_mem(f64, (usize)imageWidth * imageHeight);
		depthConverter.smIdealBinocularDisparityToDepth(leftDisparityMap, depthMap, imageWidth, imageHeight, 20.0, 60.0);


		//Discretization
		cout << "Discretization" << endl;
		u32* depthMapTrunc = allocate_mem(u32, (usize)imageWidth * imageHeight);
		u32 depthMapTruncMax = 0;
		depthConverter.smDepthDiscretization(depthMap, depthMapTrunc, &depthMapTruncMax, imageWidth, imageHeight);

		//=========== End of Depth Calculation ==================

		cout << "Initializing Camera Intrinsic" << endl;
		Common::Camera::MonocularCameraIntrinsic* cameraIntrinsic = new Common::Camera::MonocularCameraIntrinsic();
		cameraIntrinsic->fx = 450.0;
		cameraIntrinsic->fy = 375.0;
		cameraIntrinsic->cx = 450.0 / 2.0;
		cameraIntrinsic->cy = 375.0 / 2.0;
		cameraIntrinsic->dx = 1.0;
		cameraIntrinsic->dy = 1.0;

		cout << "Creating Voxels" << endl;
		DenseReconstruction::VoxelStore* voxelStore = new DenseReconstruction::PlainVoxelStore();
		voxelStore->drInitialize(128, 128, 128, -64, -64, -64, 1.0);
		DenseReconstruction::VoxelStore* voxelStoreTemp = new DenseReconstruction::PlainVoxelStore();
		voxelStoreTemp->drInitialize(128, 128, 128, -64, -64, -64, 1.0);

		cout << "Calculating Truncated Signed Distance Field" << endl;
		tsdfCalc.drIdealFirstTsdfEstimate(depthMap, imageWidth, imageHeight, cameraIntrinsic, voxelStore, 10.0);

		cout << "Converting TSDF" << endl;
		visExt.cmuTsdfBinarization(voxelStore, voxelStoreTemp);

		cout << "Converting to Mesh" << endl;
		Common::Mesh::SimpleMesh* mesh = new Common::Mesh::SimpleMesh();
		visExt.cmuVoxelMarchingCubes(voxelStore, mesh);


		cout << "F=" << mesh->f.size() << ", V=" << mesh->v.size() << endl;
		cout << "Saving As OBJ" << endl;
		visExt.cmuExportMeshToObj("C:/WR/Sayu/samples/3d-3c.obj", mesh);

		//Save PPM
		cout << "Saving PPM" << endl;
		Common::Algorithm::cmSaveAsPPM32("C:/WR/Dense-Reconstruction/samples/vs1-cb-3c.ppm", depthMapTrunc, imageWidth, imageHeight, depthMapTruncMax);
	}
	return 0;
}