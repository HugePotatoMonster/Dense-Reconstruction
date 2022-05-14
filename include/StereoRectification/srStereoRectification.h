#pragma once
#include "../Common/cmTypeDefs.h"
#include "../Common/cmMath.h"
#include "srAbstractStereoRectification.h"

namespace StereoRectification{
    class StereoRectification:public AbstractStereoRectification{
        public:
        virtual void stereoRectify(cv::Mat* imLeft, cv::Mat* imRight,
                                   Common::Camera::MonocularCameraIntrinsic* leftInt, Common::Camera::MonocularCameraIntrinsic* rightInt,
                                   Common::Camera::MonocularCameraExtrinsic* leftExt, Common::Camera::MonocularCameraExtrinsic* rightExt,
                                   OUT_ARG cv::Mat* rectifiedImLeft, OUT_ARG cv::Mat* rectifiedImRight,
                                   OUT_ARG cv::Mat* rectifiedImLeftMask, OUT_ARG cv::Mat* rectifiedImRightMask,
                                   OUT_ARG cv::Mat* homographLeft, OUT_ARG cv::Mat* homographRight,
                                   OUT_ARG cv::Mat* camCsRemapLeft, OUT_ARG cv::Mat* camCsRemapRight,
                                   OUT_ARG cv::Mat* intCsRemapLeft, OUT_ARG cv::Mat* intCsRemapRight,
                                   OUT_ARG cv::Mat* disDepthMap);
    };
}