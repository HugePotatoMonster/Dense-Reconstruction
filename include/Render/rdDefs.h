#pragma once
#include "../Common/cmTypeDefs.h"
namespace Render {
	namespace Constant {
		static i32 viewportHeight = 600;
		static i32 viewportWidth = 800;
		static const char* windowTitle = "Dense Construction";
		static const char* vertexShaderFilePath = "../../shaders/vertex.glsl";
		static const char* fragmentShaderFilePath = "../../shaders/fragment.glsl";
		static const usize shaderCompilationLogLen = 1024;
	}
	namespace Interaction{
		struct MouseRecorder{
			i32 lastX;
			i32 lastY;
			i32 curX;
			i32 curY;
			u8 initialized = 0;	
			f32 yaw = 0;
			f32 pitch = 0;
		};
	}
}