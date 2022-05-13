#include "../../include/Common/cmAlgo.h"
#include "../../include/Common/cmMath.h"
namespace Common{
    u32 Algorithm::cmBitCount(u32 x){
        u32 ret = 0;
        while (x) {
            ++ret;
            x &= x - 1;
        }
        return ret;
    }
    u32 Algorithm::cmHammingDistance(u32 x,u32 y){
        return Algorithm::cmBitCount(x^y);
    }
    u32 Algorithm::cmSaveAsPPM(std::string file_path, u8* imageData,u32 imageWidth,u32 imageHeight,u8 maxValue){
        std::ofstream imagePPMOutput(file_path);
        imagePPMOutput<<"P3"<<std::endl<<imageWidth<<" "<<imageHeight<<std::endl<<(u32)maxValue<<std::endl;
        for(u32 j=0;j<imageHeight;j++){ //Rows
            for(u32 i=0;i<imageWidth;i++){ //Cols
                imagePPMOutput<<(i32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(i32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(i32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" ";
            }
            imagePPMOutput<<std::endl;
        }
        imagePPMOutput.close();
        return 0;
    }
    u32 Algorithm::cmSaveAsPPM32(std::string file_path, u32* imageData,u32 imageWidth,u32 imageHeight,u32 maxValue){
        std::ofstream imagePPMOutput(file_path);
        imagePPMOutput<<"P3"<<std::endl<<imageWidth<<" "<<imageHeight<<std::endl<<maxValue<<std::endl;
        for(u32 j=0;j<imageHeight;j++){ //Rows
            for(u32 i=0;i<imageWidth;i++){ //Cols
                imagePPMOutput<<(u32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(u32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" "<<(u32)get_pixel(imageData,i,j,imageWidth,imageHeight)<<" ";
            }
            imagePPMOutput<<std::endl;
        }
        imagePPMOutput.close();
        return 0;
    }
    void AlgorithmCV::cmIdealEpipolarEquationByFundamentalMatrix(Common::Camera::MonocularCameraIntrinsic* imCI,
                                                               Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                                               Common::Math::Vec3* imLeftPixel, OUT_ARG cv::Mat* epiB, OUT_ARG cv::Mat* epiK){
        cv::Mat camInt;
        Common::Math::MathUtil::cmGetIntrinsicMat(imCI,&camInt);
        cv::Mat camExtRL,camExtRR,camExtTL,camExtTR;
        Common::Math::MathUtil::cmGetExtrinsicMatR(imLeftCE,&camExtRL);
        Common::Math::MathUtil::cmGetExtrinsicMatR(imRightCE,&camExtRR);
        Common::Math::MathUtil::cmGetExtrinsicMatT(imLeftCE,&camExtTL);
        Common::Math::MathUtil::cmGetExtrinsicMatT(imRightCE,&camExtTR);
        cv::Mat rotateMat,translateVec;
        rotateMat = (camExtRR*camExtRL.inv());
        translateVec = -(camExtTR-camExtTL);
        cv::Mat translateMat;
        Common::Math::MathUtil::cmGetVectorProductSkewSymMatrix(&translateVec,&translateMat);
        cv::Mat fundamentalMatrix;
        fundamentalMatrix = camInt.t().inv()*(translateMat*rotateMat)*camInt.inv();
        f64 coef1 = get_cvmat(fundamentalMatrix,0,0)*imLeftPixel->a[0]+get_cvmat(fundamentalMatrix,0,1)*imLeftPixel->a[1]+get_cvmat(fundamentalMatrix,0,2);
        f64 coef2 = get_cvmat(fundamentalMatrix,1,0)*imLeftPixel->a[0]+get_cvmat(fundamentalMatrix,1,1)*imLeftPixel->a[1]+get_cvmat(fundamentalMatrix,1,2);
        f64 coef3 = get_cvmat(fundamentalMatrix,2,0)*imLeftPixel->a[0]+get_cvmat(fundamentalMatrix,2,1)*imLeftPixel->a[1]+get_cvmat(fundamentalMatrix,2,2);
        //Equation is c1*u+c2*v+c3=0
        get_cvmatp(epiB,0,0) = 0;
        get_cvmatp(epiB,1,0) = -coef3/coef2;
        get_cvmatp(epiB,2,0) = 1;
        get_cvmatp(epiK,0,0) = -coef2;
        get_cvmatp(epiK,1,0) = coef1;
        get_cvmatp(epiK,2,0) = 1;
    }
    void AlgorithmCV::cmIdealEpipolarEquation(Common::Camera::MonocularCameraIntrinsic* imLeftCI,Common::Camera::MonocularCameraIntrinsic* imRightCI,
                                            Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                            Common::Math::Vec3* imLeftPixel, OUT_ARG cv::Mat* epiB, OUT_ARG cv::Mat* epiK){
        cv::Mat camIntL = (cv::Mat_<f64>(3,3)<<imLeftCI->fx, 0 , imLeftCI->cx,
                                                0,imLeftCI->fy,imLeftCI->cy,
                                                0,0,1);
        cv::Mat camIntR = (cv::Mat_<f64>(3,3)<<imRightCI->fx, 0 , imRightCI->cx,
                                                0,imRightCI->fy,imRightCI->cy,
                                                0,0,1);
        cv::Mat camExtRL,camExtRR,camExtTL,camExtTR;
        Common::Math::MathUtil::cmGetExtrinsicMatR(imLeftCE,&camExtRL);
        Common::Math::MathUtil::cmGetExtrinsicMatR(imRightCE,&camExtRR);
        Common::Math::MathUtil::cmGetExtrinsicMatT(imLeftCE,&camExtTL);
        Common::Math::MathUtil::cmGetExtrinsicMatT(imRightCE,&camExtTR);
        cv::Mat imLP = (cv::Mat_<f64>(3,1)<<imLeftPixel->a[0],imLeftPixel->a[1],imLeftPixel->a[2]);
        cv::Mat k = -(camIntR*camExtRR*camExtRL.inv()*camIntL.inv())*imLP;
        cv::Mat kp = -(camIntR*camExtRR*camExtRL.inv()*camIntL.inv());
        cv::Mat b = camIntR*(camExtTR-camExtTL);
        std::cout<<"ML"<<std::endl;
        for(i32 i=0;i<3;i++){
            for(i32 j=0;j<3;j++){
                std::cout<<kp.at<f64>(i,j)<<",";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
        for(i32 i=0;i<3;i++){
            epiB->at<f64>(i,0) = b.at<f64>(i,0);
            epiK->at<f64>(i,0) = k.at<f64>(i,0);
        }
        //259,219
    }
    void AlgorithmCV::cmIdealStereoRectify(cv::Mat imLeft,cv::Mat imRight,
                                        Common::Camera::MonocularCameraIntrinsic* imLeftCI,Common::Camera::MonocularCameraIntrinsic* imRightCI,
                                        Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                        OUT_ARG cv::Mat* imLeftRI, OUT_ARG cv::Mat* imRightRI,
                                        OUT_ARG cv::Mat* imLeftRE, OUT_ARG cv::Mat* imRightRE,
                                        OUT_ARG cv::Mat* ddepMat){
        cv::Mat camIntL;
        cv::Mat camIntR;
        Common::Math::MathUtil::cmGetIntrinsicMat(imLeftCI,&camIntL);
        Common::Math::MathUtil::cmGetIntrinsicMat(imRightCI,&camIntR);
        cv::Mat camExtRL,camExtRR,camExtTL,camExtTR;
        Common::Math::MathUtil::cmGetExtrinsicMatR(imLeftCE,&camExtRL);
        Common::Math::MathUtil::cmGetExtrinsicMatR(imRightCE,&camExtRR);
        Common::Math::MathUtil::cmGetExtrinsicMatT(imLeftCE,&camExtTL);
        Common::Math::MathUtil::cmGetExtrinsicMatT(imRightCE,&camExtTR);
        cv::Mat camRelR = (camExtRR * camExtRL.inv());
        cv::Mat camRelT = -(camExtTR - camExtTL);
        std::cout<<"R:"<<std::endl;
        for(i32 i=0;i<9;i++){
            std::cout<<camRelR.at<f64>(i/3,i%3)<<",";
            if(i%3==2){
                std::cout<<std::endl;
            }
        }
        std::cout<<"T:"<<std::endl;

        //camRelT.at<f64>(2,0) = 0;
        for(i32 i=0;i<3;i++){
            std::cout<<camRelT.at<f64>(i,0)<<",";
        }
        std::cout<<std::endl<<"TL:"<<std::endl;
        for(i32 i=0;i<3;i++){
            std::cout<<camExtTL.at<f64>(i,0)<<",";
        }
        std::cout<<std::endl<<"TR:"<<std::endl;
        for(i32 i=0;i<3;i++){
            std::cout<<camExtTR.at<f64>(i,0)<<",";
        }
        cv::Mat distort = OCV_IDEAL_DISTORTION;
        cv::Mat lmap1,lmap2,rmap1,rmap2;
        cv::Mat rimgl,rimgr;
        cv::stereoRectify(camIntL,distort,camIntR,distort,imLeft.size(),camRelR,camRelT,*imLeftRI,*imRightRI,*imLeftRE,*imRightRE,*ddepMat,0,-1,cv::Size());
        cv::initUndistortRectifyMap(camIntL,distort,*imLeftRI,*imLeftRE,imLeft.size(),CV_32FC2,lmap1,lmap2);
        cv::initUndistortRectifyMap(camIntR,distort,*imRightRI,*imRightRE,imLeft.size(),CV_32FC2,rmap1,rmap2);
        cv::remap(imLeft, rimgl, lmap1, lmap2, cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        cv::remap(imRight, rimgr, rmap1, rmap2, cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        cv::imshow("Left",rimgl);
        cv::waitKey(0);
        cv::imshow("right",rimgr);
        cv::waitKey(0);
    }
}