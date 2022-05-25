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
        cv::Mat camPts;
        cv::copyMakeBorder(_worldPts,camPts,0,0,0,1,CV_HAL_BORDER_CONSTANT,1);
        camPts = (extr.inv()*camPts.t()).t();
        camPts =  camPts(cv::Rect(0,0,3,camPts.size[0]));


        // 3. get depth of Voxels
        double* pix_z = new double [_coordNum];
        for (int i=0; i<_coordNum; i++){
            pix_z[i] = camPts.at<double>(i,2);
        }

        int* pix_x = new int[_coordNum];
        int* pix_y = new int[_coordNum];

        // 4. get coordinates in pixel of all Voxels 
        Utility::Algo::cam2pix(camPts,intr,_coordNum,pix_x,pix_y);

        double* depthVal = new double[_coordNum];
        double* depthDiff = new double[_coordNum];
        bool* validPts = new bool[_coordNum];
        int validNum = 0;

        // 5. get index of the Voxels that in frustum
        for (int i=0; i<_coordNum; i++){
            if (checkInFrustum(pix_x[i],pix_y[i],pix_z[i])){
                depthVal[i] = depth.at<double>(pix_y[i],pix_x[i]);
            }
            else{
                depthVal[i] = 0;
            }
           depthDiff[i] = depthVal[i] - pix_z[i];
            validPts[i] = (depthVal[i]>0) && (depthDiff[i]>=-_trunc);
            if (validPts[i]){
                validNum++;
            }
        }

    // TSDF Integration

        delete depthVal;

        double* dist = new double[_coordNum];
        int* validX = new int[validNum];
        int* validY = new int[validNum];
        int* validZ = new int[validNum];
        double* wOld = new double [validNum];
        double* tsdfVals = new double [validNum];
        double* tsdfVolNew = new double[validNum];
        double* wNew = new double[validNum];

        int cur=0;
        double* validDist = new double [validNum];
        double* validPixX = new double [validNum];
        double* validPixY = new double [validNum];
        for (int i=0; i<_coordNum; i++){
            dist[i] = min(1.0, depthDiff[i]/_trunc);
            if (validPts[i]){
                validX[cur] = _coords[i][0];
                validY[cur] = _coords[i][1];
                validZ[cur] = _coords[i][2];
                validPixX[cur] = pix_x[i];
                validPixY[cur] = pix_y[i];
                validDist[cur] = dist[i];
                wOld[cur] = _weight[validX[cur]][validY[cur]][validZ[cur]];
                tsdfVals[cur] = _tsdf[validX[cur]][validY[cur]][validZ[cur]];
                wNew[cur] = wOld[cur] + obsWeight;
                tsdfVolNew[cur] = (wOld[cur]*tsdfVals[cur]+obsWeight*validDist[cur])/wNew[cur];
                _weight[validX[cur]][validY[cur]][validZ[cur]] = wNew[cur];
                _tsdf[validX[cur]][validY[cur]][validZ[cur]] = tsdfVolNew[cur];
                cur++;
            }        
        }

        delete[] depthDiff;
        delete[] validDist;
        delete[] tsdfVolNew;

    // Color Integration

        double colorOld,colorNew;
        double BOld, GOld, ROld;
        double BNew, GNew, RNew;
        for (int i=0; i<validNum; i++){
            colorOld = _color[validX[i]][validY[i]][validZ[i]];
            BOld = floor(colorOld/(256*256)); 
            GOld = floor((colorOld-BOld*256*256)/256);
            ROld = floor(colorOld-BOld*256*256-GOld*256);

            colorNew = imgC1.at<double>(validPixY[i],validPixX[i]);

            BNew = floor(colorNew/(256*256));
            GNew = floor((colorNew-BNew*256*256)/256);
            RNew = floor(colorNew-BNew*256*256-GNew*256);

            BNew = min(255.0,round((wOld[i]*BOld+obsWeight*BNew)/wNew[i]));
            GNew = min(255.0,round((wOld[i]*GOld+obsWeight*GNew)/wNew[i]));
            RNew = min(255.0,round((wOld[i]*ROld+obsWeight*RNew)/wNew[i]));

            _color[validX[i]][validY[i]][validZ[i]] = BNew*256*256 + GNew*256 + RNew;
        }

        //TODO: Free Memory >w<
        delete[] validPixX;
        delete[] validPixY;
        delete[] wNew;
        
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