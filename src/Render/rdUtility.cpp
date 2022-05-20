#include "../../include/Render/rdUtility.h"

namespace Render {
	void GraphicsInterfaceUtility::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
		glViewport(0,0,width, height);
	}
	void GraphicsInterfaceUtility::initialize(GLFWwindow* window) {
		glfwSetFramebufferSizeCallback(window, GraphicsInterfaceUtility::framebufferSizeCallback);
	}
}