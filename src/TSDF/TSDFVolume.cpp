#include "../../include/TSDF/TSDFVolume.h"
#include "../../include/Utility/Algo.h"

#include <iostream>
#include <math.h> 

using namespace std;

namespace TSDF{

    bool TSDFVolume::checkInFrustum(int pix_x, int pix_y, double pix_z){
        return (pix_x>=0) && (pix_x<IMG_W) && (pix_y>=0) && (pix_y<IMG_H) && (pix_z>0);
    };

    void TSDFVolume::integrateTSDF(double* tsdfVals, double* validDist, double* wOld, int length, double obsWeight, double* tsdfVolNew, double* wNew){
        for (int i=0; i<length; i++){
            wNew[i] = wOld[i] + obsWeight;
            tsdfVolNew[i] = (wOld[i]*tsdfVals[i]+obsWeight*validDist[i])/wNew[i];
        }
    };


    TSDFVolume::TSDFVolume(cv::Mat bound, double voxSize){
        _bound = bound.clone();
        _voxSize = voxSize;
        _trunc = 5*voxSize;

        for (int i=0; i<3; i++){
            // cout << bound.at<double>(i,1) << "-" << bound.at<double>(i,0) << "/" << _voxSize << endl;
            _volDim[i] = ceil((bound.at<double>(i,1)-bound.at<double>(i,0))/_voxSize);
            _bound.at<double>(i,1) = bound.at<double>(i,0)+_volDim[i]*_voxSize;
        }

        for (int i=0; i<3; i++){
            _volOrigin[i] = _bound.at<double>(i,0);
        }

        _tsdf = new double** [_volDim[0]];
        _weight = new double** [_volDim[0]];
        _color = new double** [_volDim[0]];

        for (int x=0; x<_volDim[0]; x++){
            _tsdf[x] = new double* [_volDim[1]];
            _weight[x] = new double* [_volDim[1]];
            _color[x] = new double* [_volDim[1]];
            for (int y=0; y<_volDim[1]; y++){
                _tsdf[x][y] = new double [_volDim[2]];
                _weight[x][y] = new double [_volDim[2]];
                _color[x][y] = new double [_volDim[2]];
                for (int z=0; z<_volDim[2]; z++){
                    _tsdf[x][y][z] = 0;
                    _weight[x][y][z] = 0;
                    _color[x][y][z] = 0;
                }
            }
        }

        _coordNum = _volDim[0]*_volDim[1]*_volDim[2];
        _coords = new int* [_coordNum];
        for (int i=0; i<_coordNum; i++){
            _coords[i] = new int [3];
        }
        int cur = 0;
        for (int x=0; x<_volDim[0]; x++){
            for (int y=0; y<_volDim[1]; y++){ 
                for (int z=0; z<_volDim[2]; z++){
                    _coords[cur][0] = x;
                    _coords[cur][1] = y;
                    _coords[cur][2] = z;
                    cur++;
                }
            }
        }

        _worldPts = Utility::Algo::vox2world(_volOrigin,_coords,_coordNum,_voxSize);

        // cout << "x:" << _volDim[0] << endl;
        // cout << "y:" << _volDim[1] << endl;
        // cout << "z:" << _volDim[2] << endl;

        // cout << "x_o:" << _volOrigin[0] << endl;
        // cout << "y_o:" << _volOrigin[1] << endl;
        // cout << "z_o:" << _volOrigin[2] << endl;

        // cout << "_bound type:" << _bound.type() << endl;
        // cout << "_bound size:" << _bound.size() << endl;
        // cout << "_bound value:" << endl << _bound << endl;

        // _volDim ok
        // _volOrigin ok
        // _bound ok

    };

    void TSDFVolume::integrate(cv::Mat img, cv::Mat depth, cv::Mat intr, cv::Mat extr, double obsWeight){
        cv::Mat imgC1 = cv::Mat::zeros(IMG_H,IMG_W,CV_64FC1);
        for (int i=0; i<IMG_H; i++){
            for (int j=0; j<IMG_W; j++){
                imgC1.at<double>(i,j) = floor(double(img.at<cv::Vec3b>(i,j)[2])*256*256 + double(img.at<cv::Vec3b>(i,j)[1])*256 + double(img.at<cv::Vec3b>(i,j)[0]));       
            }
        }

        cv::Mat camPts = Utility::Algo::rigidTransform(_worldPts,extr.inv());
        // Utility::Log::logMat(camPts(cv::Rect(0,0,3,8)), "camPts");

        double* pix_z = new double [_coordNum];
        for (int i=0; i<_coordNum; i++){
            pix_z[i] = camPts.at<double>(i,2);
        }

        int* pix_x = new int[_coordNum];
        int* pix_y = new int[_coordNum];

        Utility::Algo::cam2pix(camPts,intr,_coordNum,pix_x,pix_y);

        // for (int i=0; i<_coordNum; i++){
        //     cout << "pix_x[" << i << "]: " << pix_x[i] << endl;
        //     cout << "pix_y[" << i << "]: " << pix_y[i] << endl;
        // }

        double* depthVal = new double[_coordNum];

        Utility::Log::logMat(depth,"depth",true,true,false);

        for (int i=0; i<_coordNum; i++){
           if (checkInFrustum(pix_x[i],pix_y[i],pix_z[i])){
                depthVal[i] = depth.at<double>(pix_y[i],pix_x[i]);
           }
           else{
               depthVal[i] = 0;
           }
        //    cout << "depthVal[" << i << "]: " << depthVal[i] << endl;
        }

        // cout << "depthVal[" << 13572 << "]: " << depthVal[13572] << endl;
        // cout << "depthVal[" << 13573 << "]: " << depthVal[13573] << endl;
        // cout << "depthVal[" << 13574 << "]: " << depthVal[13574] << endl;

    // TSDF Integration
        double* depthDiff = new double[_coordNum];
        for (int i=0; i<_coordNum; i++){
            depthDiff[i] = depthVal[i] - pix_z[i];
            // cout << "depthDiff[" << i << "]: " << depthDiff[i] << endl;
        }

        bool* validPts = new bool[_coordNum];
        int validNum = 0;
        for (int i=0; i<_coordNum; i++){
            validPts[i] = (depthVal[i]>0) && (depthDiff[i]>=-_trunc);
            if (validPts[i]){
                validNum++;
                // cout << "True index: " << i << endl;
            }
            // cout << "validPts[" << i << "]: " << validPts[i] << endl;
        }

        delete depthVal;

        double* dist = new double[_coordNum];
        for (int i=0; i<_coordNum; i++){
            dist[i] = min(1.0, depthDiff[i]/_trunc);
            // cout << "dist[" << i << "]: " << dist[i] << endl;
        }

        delete depthDiff;

        int* validX = new int[validNum];
        int* validY = new int[validNum];
        int* validZ = new int[validNum];

        int cur=0;
        double* validDist = new double [validNum];
        for (int i=0; i<_coordNum; i++){
            if (validPts[i]){
                validX[cur] = _coords[i][0];
                validY[cur] = _coords[i][1];
                validZ[cur] = _coords[i][2];
                validDist[cur] = dist[i];
                // cout << "validX[" << cur << "]: " << validX[cur] << endl;
                // cout << "validY[" << cur << "]: " << validY[cur] << endl;
                // cout << "validZ[" << cur << "]: " << validZ[cur] << endl;
                // cout << "validDist[" << cur << "]: " << validDist[cur] << endl;
                cur++;
            }        
        }

        // cout << "cur: " << cur << " vs validNum: " << validNum << endl;

        double* wOld = new double [validNum];
        double* tsdfVals = new double [validNum];

        for (int i=0; i<validNum; i++){
            wOld[i] = _weight[validX[i]][validY[i]][validZ[i]];
            tsdfVals[i] = _tsdf[validX[i]][validY[i]][validZ[i]];
            // cout << "wOld: " << wOld[i] << endl;
            // cout << "tsdfVals: " << tsdfVals[i] << endl;
        }

        double* tsdfVolNew = new double[validNum];
        double* wNew = new double[validNum];
        integrateTSDF(tsdfVals, validDist, wOld, validNum, obsWeight, tsdfVolNew, wNew);

        delete validDist;

        for (int i=0; i<validNum; i++){
            cout << "tsdfVolNew: " << tsdfVolNew[i] << endl;
            cout << "wNew: " << wNew[i] << endl;
        }

        for (int i=0; i<validNum; i++){
            _weight[validX[i]][validY[i]][validZ[i]] = wNew[i];
            _tsdf[validX[i]][validY[i]][validZ[i]] = tsdfVolNew[i];
        }

        delete tsdfVolNew;

    // Color Integration


    };
}