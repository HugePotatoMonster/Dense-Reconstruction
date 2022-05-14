#include "../../include/Test/tsTest.h"
#include "../../include/Misc/msAuxiliaryUtils.h"
#include "../../include/StereoRectification/srStereoRectification.h"

namespace Test{
    void Test::stereoRectify(){
        StereoRectification::StereoRectification* stereoRectify = new StereoRectification::StereoRectification();
        cv::Mat camItr = cv::imread("E:\\60fps_images_archieve\\scene_00_0001.png", 0);
        cv::Mat camItl = cv::imread("E:\\60fps_images_archieve\\scene_00_0002.png", 0);
        Common::Camera::MonocularCameraIntrinsic camIntb,camIntc;
        Common::Camera::MonocularCameraExtrinsic camExtb,camExtc;
        Misc::AuxiliaryUtils::msParseIntrinsic("E:\\60fps_GT_archieve\\scene_00_0001.txt",&camIntc);
        Misc::AuxiliaryUtils::msParseExtrinsic("E:\\60fps_GT_archieve\\scene_00_0001.txt",&camExtc);
        Misc::AuxiliaryUtils::msParseIntrinsic("E:\\60fps_GT_archieve\\scene_00_0002.txt",&camIntb);
        Misc::AuxiliaryUtils::msParseExtrinsic("E:\\60fps_GT_archieve\\scene_00_0002.txt",&camExtb);
        
        dbg_trace(dbg_output<<"Here In";);
        //Starts
        cv::Mat rectImL,rectImR,rectImLM,rectImRM;
        cv::Mat HL,HR,KL,KR,RL,RR,DD;
        stereoRectify->stereoRectify(&camItl,&camItr,&camIntb,&camIntc,&camExtb,&camExtc,&rectImL,&rectImR,&rectImLM,&rectImRM,&HL,&HR,&RL,&RR,&HL,&HR,&DD);

        //View
        cv::imshow("Left",rectImL);
        cv::imshow("Right",rectImR);
        cv::waitKey(0);
    }
}