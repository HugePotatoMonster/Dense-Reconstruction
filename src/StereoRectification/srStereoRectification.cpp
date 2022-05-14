#include "../../include/StereoRectification/srStereoRectification.h"

namespace StereoRectification{
    void StereoRectification::stereoRectify(cv::Mat* imLeft, cv::Mat* imRight,
                                   Common::Camera::MonocularCameraIntrinsic* leftInt, Common::Camera::MonocularCameraIntrinsic* rightInt,
                                   Common::Camera::MonocularCameraExtrinsic* leftExt, Common::Camera::MonocularCameraExtrinsic* rightExt,
                                   OUT_ARG cv::Mat* rectifiedImLeft, OUT_ARG cv::Mat* rectifiedImRight,
                                   OUT_ARG cv::Mat* rectifiedImLeftMask, OUT_ARG cv::Mat* rectifiedImRightMask,
                                   OUT_ARG cv::Mat* homographLeft, OUT_ARG cv::Mat* homographRight,
                                   OUT_ARG cv::Mat* camCsRemapLeft, OUT_ARG cv::Mat* camCsRemapRight,
                                   OUT_ARG cv::Mat* intCsRemapLeft, OUT_ARG cv::Mat* intCsRemapRight,
                                   OUT_ARG cv::Mat* disDepthMap){

        //Calculate the relative pose. That is, the right extrinsic respect to the left camera CS (I am stupid!!!)
        Common::Camera::MonocularCameraExtrinsic relExt;
        cv::Mat relR,relT;
        cv::Mat intL,intR;
        Common::Math::MathUtil::cmGetIntrinsicMat(leftInt,&intL);
        Common::Math::MathUtil::cmGetIntrinsicMat(rightInt,&intR);
        Common::Math::MathUtil::cmGetRelativeExtrinsic(leftExt,rightExt,&relExt);
        Common::Math::MathUtil::cmGetExtrinsicMatR(&relExt,&relR);
        Common::Math::MathUtil::cmGetExtrinsicMatT(&relExt,&relT);
        
        //Stereo rectify and obtain the reprojection matrix
        cv::Mat distortion = OCV_IDEAL_DISTORTION;
        cv::stereoRectify(intL,distortion,intR,distortion,imLeft->size(),relR,relT,*camCsRemapLeft,*camCsRemapRight,*intCsRemapLeft,*intCsRemapRight,*disDepthMap,0,-1,cv::Size());
        
        cv::Mat tempKL = (*intCsRemapLeft)(cv::Rect(0, 0, 3, 3));
        cv::Mat tempKR = (*intCsRemapRight)(cv::Rect(0, 0, 3, 3));
        
        //Compute homograph matrix that to be applied to the image CS
        *homographLeft = tempKL * *camCsRemapLeft * intL.inv();
        *homographRight = tempKR * *camCsRemapRight * intR.inv();

        //Wrap the image to obtain the rectified image pair
        cv::warpPerspective(*imLeft,*rectifiedImLeft,*homographLeft,imLeft->size());
        cv::warpPerspective(*imRight,*rectifiedImRight,*homographRight,imRight->size());

        //TODO: Masking 
    }
}