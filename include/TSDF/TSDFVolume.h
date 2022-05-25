#pragma once
#include "../Utility/Log.h"

#include <opencv2/opencv.hpp>

namespace TSDF{
    class TSDFVolume{
        private:
            // boundary of Volume
            cv::Mat _bound;
            // size of one Voxel
            double _voxSize;
            // trunc to calculate tsdf
            double _trunc;
            // coordinate in real world of Voxel(0,0,0)
            double _volOrigin[3];
            // size of Volume
            int _volDim[3];
            // coordinates in real world of all Voxels
            cv::Mat _worldPts;

            // store tsdf,weight,color
            double*** _tsdf;
            double*** _weight;
            double*** _color;

            // index of all Voxels (size: _coordNum*3)
            int** _coords;
            // num of all Voxels
            int _coordNum;

            bool checkInFrustum(int pix_x, int pix_y, double pix_z);
            bool checkValid();



        public:
            TSDFVolume(cv::Mat bound, double voxSiz=0.02);
            void integrate(cv::Mat img, cv::Mat depth, cv::Mat intr, cv::Mat extr, double obsWeight=1.0);
            void store(string name);
            void getObj(string name);

            // Some methods to get private values >w<
            void getDims(int* dx, int* dy, int* dz);
            void getVoxel(int x, int y, int z, double* v);
            void getColor(int x, int y, int z, double* v);
            // End of modification :)
    };
};