#pragma once
#include "rdDefs.h"
#include "../Common/cmTypeDefs.h"

namespace Render {
	class GraphicsInterfaceUtility final {
	private:
		GraphicsInterfaceUtility(){}
	public:
		static void initialize(GLFWwindow* window);
		static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
		static void getShader(const char* path, OUT_ARG std::string* shaderContent);
		static void getShaders(const char* vertexPath, const char* fragPath, OUT_ARG std::string* vertexShader, OUT_ARG std::string* fragmentShader);
		static void getShadersDefault(OUT_ARG std::string* vertexShader, OUT_ARG std::string* fragmentShader);
	};
}