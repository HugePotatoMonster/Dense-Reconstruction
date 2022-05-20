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
	};
}