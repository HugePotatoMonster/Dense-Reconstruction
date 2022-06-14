#pragma once
#include "include/Camera/ParamObtain.h"
#include "include/Utility/Reader.h"
#include "include/Utility/Algo.h"
#include "include/Utility/Log.h"
#include "include/TSDF/TSDFVolume.h"
#include "include/DenseReconstruction/MarchingCubes/drMarchingCubes.h"
#include "include/Common/Utility/cmVisExt.h"
#include "include/DepthEstimation/deDepthEstimationHelper.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <float.h>

#include <opencv2/opencv.hpp>

#include "realtime.cpp"
#include <thread>
#include <mutex>

using namespace std;

void test(){
    Utility::Reader* reader = Utility::Reader::getInstance();
    ParamHelper::ParamObtain po;

    cv::Mat intr = po.getIntrinsic_test();

    cv::Mat bound = cv::Mat::zeros(3,2,CV_64FC1);
    for (int i=0 ; i<3; i++){
        bound.at<double>(i,0) = DBL_MAX;
        bound.at<double>(i,1) = -DBL_MAX;
    }

    int inputNum = 1;

    for (int imgNo=0; imgNo<inputNum; imgNo++){

        cv::Mat depth = reader->readinDepth_test(imgNo);
        depth.convertTo(depth,CV_64FC1);
        // Utility::Log::logMat(depth,"depth");

        depth /= 1000;
        // Utility::logMat(depth,"depth");

        for (int i=0; i<IMG_H; i++){
            double* ptr = depth.ptr<double>(i);
            for (int j=0; j<IMG_W; j++){
                if(ptr[j]==65.535){
                    ptr[j]=0;
                }
            }
        }

        cv::Mat extr = reader->readinPose_test(imgNo);
        Utility::Log::logMat(extr,"extr");

        cv::Mat frustPts = Utility::Algo::getFrustum(depth,intr,extr);
        // Utility::logMat(frustPts,"frustPts");
        // cout << "w: " << frustPts.size[0] << endl;

        double minVal,maxVal;
        for (int i=0; i<3; i++){
            cv::minMaxIdx(frustPts(cv::Rect(0,i,frustPts.size[1],1)),&minVal,&maxVal);
            bound.at<double>(i,0) = min(bound.at<double>(i,0),minVal);
            bound.at<double>(i,1) = max(bound.at<double>(i,1),maxVal);
            // cout << "min: " << minVal << endl;
            // cout << "max: " << maxVal << endl;
        }
        // Utility::logMat(bound,"bound");
    }

    Utility::Log::logMat(bound,"bound");

// TEST bound
    // bound.at<double>(0,0) = -4.221064379455555;
    // bound.at<double>(0,1) = 3.867982033448718;
    // bound.at<double>(1,0) = -2.666310403125;
    // bound.at<double>(1,1) = 2.601461414461538;
    // bound.at<double>(2,0) = 0;
    // bound.at<double>(2,1) = 5.76272371304359;
    // Utility::logMat(bound,"bound");

    TSDF::TSDFVolume tsdf(bound,0.01 );

    for (int imgNo=0; imgNo<inputNum; imgNo++){
        cout << "Process imgNo: " << imgNo << endl;

        cv::Mat img = reader->readinImg_test(imgNo);

        // Utility::logMat(img,"img");
        // img.convertTo(img,CV_64FC3);

        // Utility::logMat(img,"img");

        // cout << "0,0 R: " << img.at<double>(0,0,0) << endl;
        // cout << "0,0 G: " << img.at<double>(0,0,1) << endl;
        // cout << "0,0 B: " << img.at<double>(0,0,2) << endl;

        cv::Mat depth = reader->readinDepth_test(imgNo);
        depth.convertTo(depth,CV_64FC1);
        depth /= 1000;
        for (int i=0; i<IMG_H; i++){
            double* ptr = depth.ptr<double>(i);
            for (int j=0; j<IMG_W; j++){
                if(ptr[j]==65.535){
                    ptr[j]=0;
                }
            }
        }

        cv::Mat extr = reader->readinPose_test(imgNo);

        auto startTime = chrono::system_clock::now();
        tsdf.integrate(img, depth, intr, extr);
        auto endTime = chrono::system_clock::now();

        cout << "time:" << chrono::duration_cast<chrono::seconds>(endTime - startTime).count() << endl;
    }

    tsdf.store("D:/tsdf_new_test.txt");
    // tsdf.getObj("D:/test.obj");
}

bool meshFlag=true;

void generateTSDF(Viewer* viewer,SurfaceMesh* mesh){
    Utility::Reader* reader = Utility::Reader::getInstance();
    ParamHelper::ParamObtain po;
    DepthEstimation::DepthEstimationHelper* helper = new DepthEstimation::DepthEstimationHelper();

    cv::Mat intr = po.getIntrinsic();

    cv::Mat bound = cv::Mat::zeros(3,2,CV_64FC1);
    for (int i=0 ; i<3; i++){
        bound.at<double>(i,0) = DBL_MAX;
        bound.at<double>(i,1) = -DBL_MAX;
    }

    int sampleNum = 1;
    int imgNum = 1;
    cv::Mat** depthArray = new cv::Mat*[sampleNum*imgNum];
    cv::Mat** extrArray = new cv::Mat*[sampleNum*imgNum];
    int current = 0;

    for (int sampleNo=0; sampleNo<sampleNum; sampleNo++){
        for (int imgNo=0; imgNo<imgNum; imgNo++){
            cout << "Process sampleNo: " << sampleNo << " imgNo: " << imgNo << endl;

        // depth

            // cv::Mat depth = reader->readinDepth(sampleNo,imgNo);
            // depth.convertTo(depth,CV_64FC1);

            // depth /= 1000;
            // for (int i=0; i<IMG_H; i++){
            //     double* ptr = depth.ptr<double>(i);
            //     for (int j=0; j<IMG_W; j++){
            //         if(ptr[j]==65.535){
            //             ptr[j]=0;
            //         }
            //     }
            // }

        // extr

            // depthArray[0] = new cv::Mat(480, 640, CV_64FC1);
            // extrArray[0] = new cv::Mat(4, 4, CV_64FC1);
            // helper->deIdealCalibratedDepthEstimationFilteredFromFile(
            //     "D:\\PM\\Dataset\\DenseReconstruction\\60fps_images_archieve\\scene_00_0001.png",
            //     "D:\\PM\\Dataset\\DenseReconstruction\\60fps_images_archieve\\scene_00_0002.png",
            //     "D:\\PM\\Dataset\\DenseReconstruction\\60fps_images_archieve\\scene_00_0003.png",
            //     "D:\\PM\\Dataset\\DenseReconstruction\\60fps_GT_archieve\\scene_00_0001.txt",
            //     "D:\\PM\\Dataset\\DenseReconstruction\\60fps_GT_archieve\\scene_00_0002.txt",
            //     "D:\\PM\\Dataset\\DenseReconstruction\\60fps_GT_archieve\\scene_00_0003.txt",
            //     depthArray[0], extrArray[0]);

            // Utility::Log::logMat(*extrArray[0],"extr");

            // *depthArray[current] /= -1000;

            char left_img_file_name[100];
            sprintf(left_img_file_name,"%s/%s/scene_%02d_%04d.png",Common::dataPath,Common::imgSubfolder,sampleNo,imgNo+1);  
            char mid_img_file_name[100];
            sprintf(mid_img_file_name,"%s/%s/scene_%02d_%04d.png",Common::dataPath,Common::imgSubfolder,sampleNo,imgNo+2); 
            char right_img_file_name[100];
            sprintf(right_img_file_name,"%s/%s/scene_%02d_%04d.png",Common::dataPath,Common::imgSubfolder,sampleNo,imgNo+3);

            char left_pose_file_name[100];
            sprintf(left_pose_file_name,"%s/%s/scene_%02d_%04d.txt",Common::dataPath,Common::gtSubfolder,sampleNo,imgNo+1);
            char mid_pose_file_name[100];
            sprintf(mid_pose_file_name,"%s/%s/scene_%02d_%04d.txt",Common::dataPath,Common::gtSubfolder,sampleNo,imgNo+2); 
            char right_pose_file_name[100];
            sprintf(right_pose_file_name,"%s/%s/scene_%02d_%04d.txt",Common::dataPath,Common::gtSubfolder,sampleNo,imgNo+3); 

            depthArray[current] = new cv::Mat(480, 640, CV_64FC1);
		    extrArray[current] = new cv::Mat(4, 4, CV_64FC1);

            helper->deIdealCalibratedDepthEstimationFilteredFromFile(
            left_img_file_name,
			mid_img_file_name,
			right_img_file_name,
			left_pose_file_name,
			mid_pose_file_name,
			right_pose_file_name,
			depthArray[current], extrArray[current]);

            *depthArray[current] /= -1000;
            cv::flip(*depthArray[current],*depthArray[current],1);

            cv::Mat frustPts = Utility::Algo::getFrustum(*(depthArray[current]),intr,*(extrArray[current]));

            double minVal,maxVal;
            for (int i=0; i<3; i++){
                cv::minMaxIdx(frustPts(cv::Rect(0,i,frustPts.size[1],1)),&minVal,&maxVal);
                bound.at<double>(i,0) = min(bound.at<double>(i,0),minVal);
                bound.at<double>(i,1) = max(bound.at<double>(i,1),maxVal);
            }
            current++;
        }
    }

    Utility::Log::logMat(bound,"bound");

    TSDF::TSDFVolume tsdf(bound,0.002);

    current = 0;

    for (int sampleNo=0; sampleNo<sampleNum; sampleNo++){
        for (int imgNo=0; imgNo<imgNum; imgNo++){
            cout << "Integrate sampleNo: " << sampleNo << " imgNo: " << imgNo << endl;

            cv::Mat img = reader->readinImg(sampleNo,imgNo);

        // depth

            // cv::Mat depth = reader->readinDepth(sampleNo,imgNo);
            // depth.convertTo(depth,CV_64FC1);

            // depth /= 1000;
            // for (int i=0; i<IMG_H; i++){
            //     double* ptr = depth.ptr<double>(i);
            //     for (int j=0; j<IMG_W; j++){
            //         if(ptr[j]==65.535){
            //             ptr[j]=0;
            //         }
            //     }
            // }

        // extr

            // cv::Mat extr = reader->readinPose(sampleNo,imgNo);

            // helper->deIdealCalibratedDepthEstimationFilteredFromFile(
            // left_img_file_name,
            // mid_img_file_name,
            // right_img_file_name,
            // left_pose_file_name,
            // mid_pose_file_name,
            // right_pose_file_name,
            // &depth, &extr);

            auto startTime = chrono::system_clock::now();
            // Utility::Log::logMat(*(depthArray[0]),"depth");

            tsdf.integrate(img, *(depthArray[current]), intr, *(extrArray[current]));

            auto endTime = chrono::system_clock::now();

            cout << "time:" << chrono::duration_cast<chrono::seconds>(endTime - startTime).count() << endl;

            cout << "Exporting Mesh" << endl;
            DenseReconstruction::MarchingCubes::MarchingCubesUtil* mcUtils = new  DenseReconstruction::MarchingCubes::MarchingCubesUtil();
            Common::Mesh::ColoredSimpleMesh mcMesh;

            vector<easy3d::vec3> points;
            vector<easy3d::vec3> colors;
            int vertexNum = 0;
            vector<vector<int>> faces;
            int faceNum = 0;

            mcUtils->mcGetElements(&tsdf, &points, &colors, vertexNum, &faces, faceNum);

            std::vector<io::Element> elements;
            getElements(elements, points, colors, vertexNum, faces, faceNum);

            cout << "start" << endl;

            if(!meshFlag){
                viewer->delete_model(mesh);
            }

            mesh = elements2Mesh(elements, to_string(sampleNo)+to_string(sampleNum));

            viewer->add_model(mesh);

            viewer->update();

            meshFlag = false;

            cout << "end" << endl;
            current++;
        }
    }
}

int main_2() {
    test();
    // generateTSDF();
    return 0;
}
