#include "../../include/TSDF/TSDFVolume.h"

#include <iostream>
#include <math.h> 

using namespace std;

namespace TSDF{

    TSDFVolume::TSDFVolume(cv::Mat bound, double voxSize){
        _bound = bound.clone();
        _voxSize = voxSize;

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
                }
            }
        }

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

}