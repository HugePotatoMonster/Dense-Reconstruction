#include "../../include/TSDF/TSDFVolume.h"
#include "../../include/Utility/Algo.h"

#include <iostream>
#include <fstream>
#include <math.h> 

using namespace std;

namespace TSDF{

    bool TSDFVolume::checkInFrustum(int pix_x, int pix_y, double pix_z){
        return (pix_x>=0) && (pix_x<IMG_W) && (pix_y>=0) && (pix_y<IMG_H) && (pix_z>0);
    };

    TSDFVolume::TSDFVolume(cv::Mat bound, double voxSize){
        cout << "TSDFVolume initing......" << endl;
        _bound = bound.clone();
        _voxSize = voxSize;
        _trunc = 5*voxSize;

        for (int i=0; i<3; i++){
            _volDim[i] = ceil((bound.at<double>(i,1)-bound.at<double>(i,0))/_voxSize);
        }

        cout << "size: " << _volDim[0] << "*" << _volDim[1] << "*" <<  _volDim[2] << endl; 

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
                    _tsdf[x][y][z] = 1;
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

        _worldPts = cv::Mat::zeros(_coordNum,3,CV_64FC1);
        for (int i=0; i<_coordNum; i++){
            for (int j=0 ; j<3; j++){
                _worldPts.at<double>(i,j) = _volOrigin[j] + (_voxSize*double(_coords[i][j]));
            }
        } 


        cout << "Init finished." << endl;
    };

    void TSDFVolume::integrate(cv::Mat img, cv::Mat depth, cv::Mat intr, cv::Mat extr, double obsWeight){

    // preparation

        // 1. convert 3C color to 1C
        cv::Mat imgC1 = cv::Mat::zeros(IMG_H,IMG_W,CV_64FC1);
        for (int i=0; i<IMG_H; i++){
            for (int j=0; j<IMG_W; j++){
                imgC1.at<double>(i,j) = floor(double(img.at<cv::Vec3b>(i,j)[2])*256*256 + double(img.at<cv::Vec3b>(i,j)[1])*256 + double(img.at<cv::Vec3b>(i,j)[0]));       
            }
        }

        // 2. get coordinates in camera of all Voxels
        // auto startTime = chrono::system_clock::now();
        cv::Mat camPts;
        cv::copyMakeBorder(_worldPts,camPts,0,0,0,1,CV_HAL_BORDER_CONSTANT,1);
        camPts = (extr.inv()*camPts.t()).t();
        camPts =  camPts(cv::Rect(0,0,3,camPts.size[0]));
        // auto endTime = chrono::system_clock::now();
        // cout << "time to calculate camPts:" << chrono::duration_cast<chrono::seconds>(endTime - startTime).count() << endl;

        // 3. get index of the Voxels that in frustum
        int* pix_x = new int[_coordNum];
        int* pix_y = new int[_coordNum];
        double* pix_z = new double [_coordNum];

        double depthVal, depthDiff;
        int validNum = 0;
        camPts = intr*camPts.t();

        std::vector<int> validIndex;
        std::vector<double> dist;

        // startTime = chrono::system_clock::now();
        for (int i=0; i<_coordNum; i++){
            pix_x[i] = round(camPts.at<double>(0,i)/camPts.at<double>(2,i));
            pix_y[i] = round(camPts.at<double>(1,i)/camPts.at<double>(2,i));
            pix_z[i] = camPts.at<double>(2,i);
            if (checkInFrustum(pix_x[i],pix_y[i],pix_z[i])){
                depthVal = depth.at<double>(pix_y[i],pix_x[i]);
            }
            else{
                depthVal = 0;
            }
            depthDiff = depthVal - pix_z[i];
            if ((depthVal>0) && (depthDiff>=-_trunc)){
                validIndex.push_back(i);
                dist.push_back(min(1.0, depthDiff/_trunc));
                validNum++;
            }
        }
        // endTime = chrono::system_clock::now();
        // cout << "time to find frustum:" << chrono::duration_cast<chrono::seconds>(endTime - startTime).count() << endl;

    // TSDF Integration

        int validX, validY, validZ;
        double wOld, tsdfVals, tsdfVolNew, wNew;

        int cur=0;

        int validPixX, validPixY;

        double colorOld,colorNew;
        double BOld, GOld, ROld;
        double BNew, GNew, RNew;

        // startTime = chrono::system_clock::now();
        for (int i=0; i<validNum; i++){
            int index = validIndex[i];
            validX = _coords[index][0];
            validY = _coords[index][1];
            validZ = _coords[index][2];
            validPixX = pix_x[index];
            validPixY = pix_y[index];
            wOld = _weight[validX][validY][validZ];
            tsdfVals = _tsdf[validX][validY][validZ];
            wNew = wOld + obsWeight;
            tsdfVolNew = (wOld*tsdfVals+obsWeight*dist[i])/wNew;
            _weight[validX][validY][validZ] = wNew;
            _tsdf[validX][validY][validZ] = tsdfVolNew;   

            colorOld = _color[validX][validY][validZ];
            BOld = floor(colorOld/(256*256)); 
            GOld = floor((colorOld-BOld*256*256)/256);
            ROld = floor(colorOld-BOld*256*256-GOld*256);

            colorNew = imgC1.at<double>(validPixY,validPixX);

            BNew = floor(colorNew/(256*256));
            GNew = floor((colorNew-BNew*256*256)/256);
            RNew = floor(colorNew-BNew*256*256-GNew*256);

            BNew = min(255.0,round((wOld*BOld+obsWeight*BNew)/wNew));
            GNew = min(255.0,round((wOld*GOld+obsWeight*GNew)/wNew));
            RNew = min(255.0,round((wOld*ROld+obsWeight*RNew)/wNew));

            _color[validX][validY][validZ] = BNew*256*256 + GNew*256 + RNew;   
        }
        // endTime = chrono::system_clock::now();
        // cout << "time integrate:" << chrono::duration_cast<chrono::seconds>(endTime - startTime).count() << endl;
    };

    void TSDFVolume::store(string name){
        ofstream fileOut(name);
        fileOut << "dim" << endl;
        fileOut << _volDim[0] << " " << _volDim[1] << " " << _volDim[2] << endl;

        fileOut << "vox size" << endl;
        fileOut << _voxSize << endl;

        fileOut << "vox origin" << endl;
        fileOut << _volOrigin[0] << " " << _volOrigin[1] << " " << _volOrigin[2] << endl;

        fileOut << "TSDF Color Weight" << endl;
        int cur = 0;
        for (int x=0; x<_volDim[0]; x++){
            for (int y=0; y<_volDim[1]; y++){
                for (int z=0; z<_volDim[2]; z++){
                    if (cur % 50000 == 0) {
                        cout << cur << "/" << _coordNum << endl;
                    }
                    double color = _color[x][y][z];
                    int B = floor(color/(256*256)); 
                    int G = floor((color-B*256*256)/256);
                    int R = floor(color-B*256*256-G*256);
                    fileOut << _tsdf[x][y][z] << " " << R << " " << G << " " << B << " " << _weight[x][y][z] << endl;
                    cur++;
                }
            }
        }
    }

    void TSDFVolume::getObj(string name){
        
    }

    // Some methods to get private values TvT
    void TSDFVolume::getDims(int* x, int* y, int* z) {
        *x = _volDim[0];
        *y = _volDim[1];
        *z = _volDim[2];
    }
    void TSDFVolume::getVoxel(int x, int y, int z, double* v) {
        *v = _tsdf[x][y][z];
    }
    void TSDFVolume::getColor(int x, int y, int z, double* v) {
        *v = _color[x][y][z];
    }
    // End of modification OvO
}