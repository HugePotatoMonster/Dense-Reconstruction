#include "include/Camera/ParamObtain.h"
#include "include/Utility/Reader.h"
#include "include/Utility/Algo.h"
#include "include/Utility/Log.h"
#include "include/TSDF/TSDFVolume.h"
#include "include/DenseReconstruction/MarchingCubes/drMarchingCubes.h"
#include "include/Common/Utility/cmVisExt.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <float.h>

#include <opencv2/opencv.hpp>

using namespace std;

void test(){
    Utility::Reader* reader = Utility::Reader::getInstance();
    Camera::ParamObtain po;

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

    tsdf.store("D:/tsdf.txt");
    // tsdf.getObj("D:/test.obj");
}

void generateTSDF(){
    Utility::Reader* reader = Utility::Reader::getInstance();
    Camera::ParamObtain po;

    cv::Mat intr = po.getIntrinsic();

    cv::Mat bound = cv::Mat::zeros(3,2,CV_64FC1);
    for (int i=0 ; i<3; i++){
        bound.at<double>(i,0) = DBL_MAX;
        bound.at<double>(i,1) = -DBL_MAX;
    }

    int sampleNum = 1;
    int imgNum = 5;

    for (int sampleNo=0; sampleNo<sampleNum; sampleNo++){
        for (int imgNo=0; imgNo<imgNum; imgNo++){
            cout << "Process sampleNo: " << sampleNo << " imgNo: " << imgNo << endl;

            cv::Mat depth = reader->readinDepth(sampleNo,imgNo);
            depth.convertTo(depth,CV_64FC1);
            // Utility::Log::logMat(depth,"depth");

            depth /= 1000;

            reader->readinImg(sampleNo,imgNo);

            for (int i=0; i<IMG_H; i++){
                double* ptr = depth.ptr<double>(i);
                for (int j=0; j<IMG_W; j++){
                    if(ptr[j]==65.535){
                        ptr[j]=0;
                    }
                }
            }

            cv::Mat extr = reader->readinPose(sampleNo,imgNo);
            // Utility::Log::logMat(extr,"extr");

            cv::Mat frustPts = Utility::Algo::getFrustum(depth,intr,extr);

            double minVal,maxVal;
            for (int i=0; i<3; i++){
                cv::minMaxIdx(frustPts(cv::Rect(0,i,frustPts.size[1],1)),&minVal,&maxVal);
                bound.at<double>(i,0) = min(bound.at<double>(i,0),minVal);
                bound.at<double>(i,1) = max(bound.at<double>(i,1),maxVal);
            }
        }

        Utility::Log::logMat(bound,"bound");

        TSDF::TSDFVolume tsdf(bound,0.01);

        for (int sampleNo=0; sampleNo<sampleNum; sampleNo++){
            for (int imgNo=0; imgNo<imgNum; imgNo++){
                cout << "Inregrate sampleNo: " << sampleNo << " imgNo: " << imgNo << endl;

                cv::Mat img = reader->readinImg(sampleNo,imgNo);

                cv::Mat depth = reader->readinDepth(sampleNo,imgNo);
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

                cv::Mat extr = reader->readinPose(sampleNo,imgNo);

                auto startTime = chrono::system_clock::now();
                tsdf.integrate(img, depth, intr, extr);
                auto endTime = chrono::system_clock::now();

                cout << "time:" << chrono::duration_cast<chrono::seconds>(endTime - startTime).count() << endl;
            }
        }
        
        //tsdf.store("E:/tsdf_p_fixed.txt");
        // tsdf.getObj("D:/test_p.obj");
        cout << "Exporting Mesh" << endl;
        Common::Util::VisualizationExt visExt;
        DenseReconstruction::MarchingCubes::MarchingCubesUtil* mcUtils = new  DenseReconstruction::MarchingCubes::MarchingCubesUtil();
        Common::Mesh::SimpleMesh mcMesh;
        mcUtils->mcConvertToMesh(&tsdf, &mcMesh);
        visExt.cmuExportMeshToObj("E:\\a.obj", &mcMesh);
    }

}

int main_2() {
    test();
    // generateTSDF();
    return 0;
}
