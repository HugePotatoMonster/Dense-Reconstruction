#include "../../include/StereoRectification/srStereoRectification.h"

namespace StereoRectification{
    void StereoRectification::assembleReprojectionMatrix(cv::Mat* intrinsic, cv::Mat* camReproj, cv::Mat* extrinsic, cv::Mat* output) {
        //This part is referenced from openMVS (by cdcseacave)
        //Modifications are done for integration.
        *output = cv::Mat(3, 4, cv::DataType<f64>::type);
        cv::Mat M(*output, cv::Rect(0, 0, 3, 3));
        cv::Mat(*intrinsic * *camReproj).copyTo(M); //3x3
        (*output).col(3) = M * cv::Mat(-*extrinsic); //3x1

    }
    void StereoRectification::stereoRectify(cv::Mat* imLeft, cv::Mat* imRight,
                                   Common::Camera::MonocularCameraIntrinsic* leftInt, Common::Camera::MonocularCameraIntrinsic* rightInt,
                                   Common::Camera::MonocularCameraExtrinsic* leftExt, Common::Camera::MonocularCameraExtrinsic* rightExt,
                                   OUT_ARG cv::Mat* rectifiedImLeft, OUT_ARG cv::Mat* rectifiedImRight,
                                   OUT_ARG cv::Mat* rectifiedImLeftMask, OUT_ARG cv::Mat* rectifiedImRightMask,
                                   OUT_ARG cv::Mat* homographLeft, OUT_ARG cv::Mat* homographRight,
                                   OUT_ARG cv::Mat* camCsRemapLeft, OUT_ARG cv::Mat* camCsRemapRight,
                                   OUT_ARG cv::Mat* intCsRemapLeft, OUT_ARG cv::Mat* intCsRemapRight,
                                   OUT_ARG cv::Mat* disDepthMap){
        //This part is referenced from openMVS (by cdcseacave)
        //Modifications are done for integration.
        
        //Calculate the relative pose. That is, the right extrinsic respect to the left camera CS (I am stupid!!!)
        Common::Camera::MonocularCameraExtrinsic relExt;
        cv::Mat relR,relT;
        cv::Mat intL,intR;
        cv::Mat camLT, camRT;
        cv::Mat camLR, camRR;
        Common::Math::MathUtil::cmGetIntrinsicMat(leftInt,&intL);
        Common::Math::MathUtil::cmGetIntrinsicMat(rightInt,&intR);
        Common::Math::MathUtil::cmGetRelativeExtrinsic(leftExt,rightExt,&relExt);
        Common::Math::MathUtil::cmGetExtrinsicMatR(&relExt,&relR);
        Common::Math::MathUtil::cmGetExtrinsicMatT(&relExt,&relT);

        Common::Math::MathUtil::cmGetExtrinsicMatT(leftExt, &camLT);
        Common::Math::MathUtil::cmGetExtrinsicMatT(rightExt, &camRT);
        Common::Math::MathUtil::cmGetExtrinsicMatR(leftExt, &camLR);
        Common::Math::MathUtil::cmGetExtrinsicMatR(rightExt, &camRR);

        cv::Mat lmap1, lmap2, rmap1, rmap2;

       
        if (!useOpenCV) {
            //Fusiello Rectification
            //Compute R_rect
            cv::Mat v1 = camRT - camLT; //Baseline
            cv::Mat v2d = (cv::Mat_<f64>(3, 1) << leftExt->r[2][0], leftExt->r[2][1], leftExt->r[2][2]);
            cv::Mat v2dskew;
            Common::Math::MathUtil::cmGetVectorProductSkewSymMatrix(&v2d, &v2dskew);
            cv::Mat v2 = v2dskew * v1; //Forward
            cv::Mat v1skew;
            Common::Math::MathUtil::cmGetVectorProductSkewSymMatrix(&v1, &v1skew);
            cv::Mat v3 = v1skew * v2; //Top
            cv::Mat nR = cv::Mat(3, 3, CV_64FC1);
            f64 v1l = 0, v2l = 0, v3l = 0;
            for (i32 i = 0; i < 3; i++) {
                v1l += get_cvmat(v1, i, 0) * get_cvmat(v1, i, 0);
                v2l += get_cvmat(v2, i, 0) * get_cvmat(v2, i, 0);
                v3l += get_cvmat(v3, i, 0) * get_cvmat(v3, i, 0);
            }
            for (i32 i = 0; i < 3; i++) {
                get_cvmat(nR, 0, i) = get_cvmat(v1, i, 0) / sqrt(v1l);
                get_cvmat(nR, 1, i) = get_cvmat(v2, i, 0) / sqrt(v2l);
                get_cvmat(nR, 2, i) = get_cvmat(v3, i, 0) / sqrt(v3l);
            }

            // Obtain newly updated intrinsic
            cv::Mat camIntLN, camIntRN;
            camIntLN = intL.clone();
            camIntRN = intR.clone();
            get_cvmat(camIntLN, 0, 1) = get_cvmat(camIntRN, 0, 1) = 0;
            get_cvmat(camIntLN, 1, 1) = get_cvmat(camIntRN, 1, 1) = (get_cvmat(intL, 1, 1) + get_cvmat(intR, 1, 1)) / 2;

            //Obtain the rotation matrix
            cv::Mat rL, rR;
            rL = nR * camLR.t();
            rR = nR * camRR.t();

            //Reprojection
            cv::Mat pL, pR;
            assembleReprojectionMatrix(&camIntLN, &rL, &camLT, &pL);
            assembleReprojectionMatrix(&camIntRN, &rR, &camRT, &pR);
            *camCsRemapLeft = rL.clone();
            *camCsRemapRight = rR.clone();
            *intCsRemapLeft = camIntLN.clone();
            *intCsRemapRight = camIntRN.clone();
        
            // End of Fusiello Rectification
        }
        else {
            //Stereo rectify and obtain the reprojection matrix using OpenCV
            cv::Mat distortion = OCV_IDEAL_DISTORTION;
            cv::stereoRectify(intL,distortion,intR,distortion,imLeft->size(),relR,relT,*camCsRemapLeft,*camCsRemapRight,*intCsRemapLeft,*intCsRemapRight,*disDepthMap,0,-1,cv::Size());
            
            
            cv::initUndistortRectifyMap(intL, distortion, *camCsRemapLeft, *intCsRemapLeft, imLeft->size(), CV_32FC2, lmap1, lmap2);
            cv::initUndistortRectifyMap(intR, distortion, *camCsRemapRight, *intCsRemapRight, imLeft->size(), CV_32FC2, rmap1, rmap2);
            cv::remap(*imLeft, *rectifiedImLeft, lmap1, lmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            cv::remap(*imRight, *rectifiedImRight, rmap1, rmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        
        if (!useOpenCV) {
            cv::Mat tempKL = (*intCsRemapLeft)(cv::Rect(0, 0, 3, 3));
            cv::Mat tempKR = (*intCsRemapRight)(cv::Rect(0, 0, 3, 3));

            //Compute homograph matrix that to be applied to the image CS
            *homographLeft = tempKL * *camCsRemapLeft * intL.inv();
            *homographRight = tempKR * *camCsRemapRight * intR.inv();

            //Wrap the image to obtain the rectified image pair
            cv::warpPerspective(*imLeft, *rectifiedImLeft, *homographLeft, imLeft->size());
            cv::warpPerspective(*imRight, *rectifiedImRight, *homographRight, imRight->size());

            //TODO: Masking 
            cv::Point2d plt = { 0,0 };
            cv::Point2d plb = { 0, (f64)imLeft->rows };
            cv::Point2d prt = { (f64)imLeft->cols, 0 };
            cv::Point2d prb = { (f64)imLeft->cols, (f64)imLeft->rows };
            std::vector<cv::Point2d> maskConvex = { plt,plb,prb,prt };
            std::vector<cv::Point2d> warpedMaskConvexL(4);
            std::vector<cv::Point2d> warpedMaskConvexR(4);
            cv::perspectiveTransform(maskConvex, warpedMaskConvexL, *homographLeft);
            cv::perspectiveTransform(maskConvex, warpedMaskConvexR, *homographRight);
            std::vector<std::vector<cv::Point2i>> contourL(1), contourR(1);
            for (i32 i = 0; i < 4; i++) {
                cv::Point2i tmpL = { (i32)warpedMaskConvexL[i].x, (i32)warpedMaskConvexL[i].y };
                cv::Point2i tmpR = { (i32)warpedMaskConvexR[i].x, (i32)warpedMaskConvexR[i].y };
                contourL.front().emplace_back(tmpL);
                contourR.front().emplace_back(tmpR);
            }
            *rectifiedImLeftMask = cv::Mat(imLeft->size(), imLeft->type());
            *rectifiedImRightMask = cv::Mat(imLeft->size(), imLeft->type());
            cv::drawContours(*rectifiedImLeftMask, contourL, 0, cv::Scalar(255), cv::FILLED);
            cv::drawContours(*rectifiedImRightMask, contourR, 0, cv::Scalar(255), cv::FILLED);

            //TODO: Disparity to Depth Matrix
        }
        else {
            //TODO: Masking for OpenCV Stereo Rectify
            cv::Mat identity = (cv::Mat_<f64>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            cv::Point2d plt = { 0,0 };
            cv::Point2d plb = { 0, (f64)imLeft->rows };
            cv::Point2d prt = { (f64)imLeft->cols, 0 };
            cv::Point2d prb = { (f64)imLeft->cols, (f64)imLeft->rows };
            std::vector<cv::Point2d> maskConvex = { plt,plb,prb,prt };
            std::vector<cv::Point2d> warpedMaskConvexL(4);
            std::vector<cv::Point2d> warpedMaskConvexR(4);
            cv::perspectiveTransform(maskConvex, warpedMaskConvexL, identity);
            cv::perspectiveTransform(maskConvex, warpedMaskConvexR, identity);
            std::vector<std::vector<cv::Point2i>> contourL(1), contourR(1);
            for (i32 i = 0; i < 4; i++) {
                cv::Point2i tmpL = { (i32)warpedMaskConvexL[i].x, (i32)warpedMaskConvexL[i].y };
                cv::Point2i tmpR = { (i32)warpedMaskConvexR[i].x, (i32)warpedMaskConvexR[i].y };
                contourL.front().emplace_back(tmpL);
                contourR.front().emplace_back(tmpR);
            }
            cv::Mat tempLeft, tempRight;
            tempLeft = cv::Mat(imLeft->size(), imLeft->type());
            tempRight = cv::Mat(imLeft->size(), imLeft->type());
            cv::drawContours(tempLeft, contourL, 0, cv::Scalar(255), cv::FILLED);
            cv::drawContours(tempRight, contourR, 0, cv::Scalar(255), cv::FILLED);
            cv::remap(tempLeft, *rectifiedImLeftMask, lmap1, lmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            cv::remap(tempRight, *rectifiedImRightMask, rmap1, rmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        }
        
    }
}