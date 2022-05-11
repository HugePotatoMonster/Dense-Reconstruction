#include "../../include/Common/cmMath.h"

namespace Common {
	namespace Math {
		static void MathUtil::cmMat3Inverse(Mat3* inMat, OUT_ARG Mat3* outMat) {
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
		static void MathUtil::cmMat3Determinant(Mat3* inMat, OUT_ARG f64* outDet) {
			*outDet = 0;
			*outDet += inMat->a[0] * inMat->a[4] * inMat->a[8];
			*outDet += inMat->a[1] * inMat->a[5] * inMat->a[6];
			*outDet += inMat->a[2] * inMat->a[3] * inMat->a[7];
			*outDet -= inMat->a[2] * inMat->a[4] * inMat->a[6];
			*outDet -= inMat->a[1] * inMat->a[3] * inMat->a[8];
			*outDet -= inMat->a[0] * inMat->a[5] * inMat->a[7];
		}
		static void MathUtil::cmGetExtrinsicMat(Common::Camera::MonocularCameraExtrinsic* camExt, OUT_ARG Mat3* outMat) {

		}
	}
}