#include "../../include/Common/cmMath.h"

namespace Common {
	namespace Math {
		void MathUtil::cmMat3Inverse(Mat3* inMat, OUT_ARG Mat3* outMat) {
			f64 inMatDet;
			cmMat3Determinant(inMat, &inMatDet);
			outMat->a[0] = +det2(inMat->a[4], inMat->a[5], inMat->a[7], inMat->a[8]) / inMatDet;
			outMat->a[1] = -det2(inMat->a[3], inMat->a[5], inMat->a[6], inMat->a[8]) / inMatDet;
			outMat->a[2] = +det2(inMat->a[3], inMat->a[4], inMat->a[6], inMat->a[7]) / inMatDet;
			outMat->a[3] = -det2(inMat->a[1], inMat->a[2], inMat->a[7], inMat->a[8]) / inMatDet;
			outMat->a[4] = +det2(inMat->a[0], inMat->a[2], inMat->a[6], inMat->a[8]) / inMatDet;
			outMat->a[5] = -det2(inMat->a[0], inMat->a[1], inMat->a[6], inMat->a[7]) / inMatDet;
			outMat->a[6] = +det2(inMat->a[1], inMat->a[2], inMat->a[4], inMat->a[5]) / inMatDet;
			outMat->a[7] = -det2(inMat->a[0], inMat->a[2], inMat->a[3], inMat->a[5]) / inMatDet;
			outMat->a[8] = +det2(inMat->a[0], inMat->a[1], inMat->a[3], inMat->a[4]) / inMatDet;
		}
		void MathUtil::cmMat3Determinant(Mat3* inMat, OUT_ARG f64* outDet) {
			*outDet = 0;
			*outDet += inMat->a[0] * inMat->a[4] * inMat->a[8];
			*outDet += inMat->a[1] * inMat->a[5] * inMat->a[6];
			*outDet += inMat->a[2] * inMat->a[3] * inMat->a[7];
			*outDet -= inMat->a[2] * inMat->a[4] * inMat->a[6];
			*outDet -= inMat->a[1] * inMat->a[3] * inMat->a[8];
			*outDet -= inMat->a[0] * inMat->a[5] * inMat->a[7];
		}
		void MathUtil::cmGetExtrinsicMatR(Common::Camera::MonocularCameraExtrinsic* camExt, OUT_ARG cv::Mat* outMat) {
			*outMat = (cv::Mat_<f64>(3,3)<<camExt->r[0][0],camExt->r[0][1],camExt->r[0][2],
											camExt->r[1][0],camExt->r[1][1],camExt->r[1][2],
											camExt->r[2][0],camExt->r[2][1],camExt->r[2][2]); 
		}
		void MathUtil::cmGetExtrinsicMatT(Common::Camera::MonocularCameraExtrinsic* camExt, OUT_ARG cv::Mat* outMat) {
			*outMat = (cv::Mat_<f64>(3,1)<<camExt->t[0],camExt->t[1],camExt->t[2]);
		}
		void MathUtil::cmGetIntrinsicMat(Common::Camera::MonocularCameraIntrinsic* camExt, OUT_ARG cv::Mat* outMat) {
			*outMat = (cv::Mat_<f64>(3,3)<<camExt->fx,0,camExt->cx,
										   0,camExt->fy,camExt->cy,
										   0,0,1); 
		}
		void MathUtil::cmGetIntrinsicMatYFlip(Common::Camera::MonocularCameraIntrinsic* camExt,f64 h, OUT_ARG cv::Mat* outMat) {
			*outMat = (cv::Mat_<f64>(3,3)<<camExt->fx,0,camExt->cx,
										   0,-camExt->fy,-camExt->cy+h,
										   0,0,1); 
		}
		void MathUtil::cmGetVectorProductSkewSymMatrix(cv::Mat* inMat,OUT_ARG cv::Mat* outMat){
			*outMat = (cv::Mat_<f64>(3,3)<<0,-inMat->at<f64>(2,0),inMat->at<f64>(1,0),
											inMat->at<f64>(2,0),0,-inMat->at<f64>(0,0),
											-inMat->at<f64>(1,0),inMat->at<f64>(0,0),0);
		}
		void MathUtil::cmGetRelativeExtrinsic(Common::Camera::MonocularCameraExtrinsic* camLeft, Common::Camera::MonocularCameraExtrinsic* camRight, Common::Camera::MonocularCameraExtrinsic* camRel){
			//Returns the relative transformation from left extrinsic to right extrinsic
			//Namely, the right extrinsic in the left camera CS
			cv::Mat leftR,leftT,rightR,rightT;
			cmGetExtrinsicMatR(camLeft,&leftR);
			cmGetExtrinsicMatR(camRight,&rightR);
			cmGetExtrinsicMatT(camLeft,&leftT);
			cmGetExtrinsicMatT(camRight,&rightT);
			cv::Mat relR,relT;
			//This can be obtained from homogeneous extrinsic (4x4) matrix 
			relR = rightR*leftR.inv();
			relT = leftR * (rightT-leftT);
			for(i32 i=0;i<3;i++){
				for(i32 j=0;j<3;j++){
					camRel->r[i][j] = get_cvmat(relR,i,j);
				}
				camRel->t[i] = get_cvmat(relT,i,0);
			}
		}
	}
}