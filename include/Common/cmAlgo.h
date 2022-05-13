#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include "../../include/Common/cmTypeDefs.h"

namespace Common{
    class Algorithm{
        public:
        static u32 cmBitCount(u32 x);
        static u32 cmHammingDistance(u32 x, u32 y);
        static u32 cmSaveAsPPM(std::string file_path, u8* imageData,u32 imageWidth,u32 imageHeight,u8 maxValue=255);
        static u32 cmSaveAsPPM32(std::string file_path, u32* imageData,u32 imageWidth,u32 imageHeight,u32 maxValue=255);
    };
    class AlgorithmCV{
        public:
        static void cmIdealStereoRectify(cv::Mat imLeft,cv::Mat imRight,
                                        Common::Camera::MonocularCameraIntrinsic* imLeftCI,Common::Camera::MonocularCameraIntrinsic* imRightCI,
                                        Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                        OUT_ARG cv::Mat* imLeftRI, OUT_ARG cv::Mat* imRightRI,
                                        OUT_ARG cv::Mat* imLeftRE, OUT_ARG cv::Mat* imRightRE,
                                        OUT_ARG cv::Mat* ddepMat);
        static void cmIdealEpipolarEquation(Common::Camera::MonocularCameraIntrinsic* imLeftCI,Common::Camera::MonocularCameraIntrinsic* imRightCI,
                                            Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                            Common::Math::Vec3* imLeftPixel, OUT_ARG cv::Mat* epiB, OUT_ARG cv::Mat* epiK);
        static void cmIdealEpipolarEquationByFundamentalMatrix(Common::Camera::MonocularCameraIntrinsic* imCI,
                                                               Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                                               Common::Math::Vec3* imLeftPixel, OUT_ARG cv::Mat* epiB, OUT_ARG cv::Mat* epiK);
    };
};