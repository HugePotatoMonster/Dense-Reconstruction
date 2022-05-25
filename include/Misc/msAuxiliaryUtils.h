#pragma once
#include "../Common/cmTypeDefs.h"
namespace Misc {
	class AuxiliaryUtils {
	public:
		static void msParseExtrinsic(std::string file, OUT_ARG Common::Camera::MonocularCameraExtrinsic* camEx);
		static void msParseIntrinsic(std::string file, OUT_ARG Common::Camera::MonocularCameraIntrinsic* camIn);
	};
}