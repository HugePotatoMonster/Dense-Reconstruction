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
    void AlgorithmCV::cmIdealStereoRectify(cv::Mat imLeft,cv::Mat imRight,
                                        Common::Camera::MonocularCameraIntrinsic* imLeftCI,Common::Camera::MonocularCameraIntrinsic* imRightCI,
                                        Common::Camera::MonocularCameraExtrinsic* imLeftCE,Common::Camera::MonocularCameraExtrinsic* imRightCE,
                                        OUT_ARG cv::Mat* imLeftRI, OUT_ARG cv::Mat* imRightRI,
                                        OUT_ARG cv::Mat* imLeftRE, OUT_ARG cv::Mat* imRightRE,
                                        OUT_ARG cv::Mat* ddepMat){
        cv::Mat camIntL = (cv::Mat_<f64>(3,3)<<imLeftCI->fx, 0 , imLeftCI->cx,
                                                0,imLeftCI->fy,imRightCI->cy,
                                                0,0,1);
        cv::Mat camIntR = (cv::Mat_<f64>(3,3)<<imRightCI->fx, 0 , imRightCI->cx,
                                                0,imRightCI->fy,imRightCI->cy,
                                                0,0,1);
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