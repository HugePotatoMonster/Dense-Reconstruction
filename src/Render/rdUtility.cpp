#include "../../include/Render/rdUtility.h"

namespace Render {
	void GraphicsInterfaceUtility::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
		glViewport(0,0,width, height);
	}
	void GraphicsInterfaceUtility::initialize(GLFWwindow* window) {
		glfwSetFramebufferSizeCallback(window, GraphicsInterfaceUtility::framebufferSizeCallback);
	}
	void GraphicsInterfaceUtility::getShader(const char* path, OUT_ARG std::string* content){
		using namespace std;
		ifstream fs(path);
		try{
			std::string str((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>()); 
			*content = str;
		}catch(std::exception e){
			*content = "";
		}
	}
	void GraphicsInterfaceUtility::getShaders(const char* vertexPath, const char* fragPath, OUT_ARG std::string* vertexShader, OUT_ARG std::string* fragmentShader){
		getShader(vertexPath,vertexShader);
		getShader(fragPath,fragmentShader);
		if(*vertexShader == ""){
			dbg_toutput("Vertex shader cannot be loaded.");
			pr_assert(false);
		}
		if(*fragmentShader == ""){
			dbg_toutput("Fragment shader cannot be loaded.");
			pr_assert(false);
		}
	}
	void GraphicsInterfaceUtility::getShadersDefault(OUT_ARG std::string* vertexShader, OUT_ARG std::string* fragmentShader){
		using namespace Render::Constant;
		getShaders(vertexShaderFilePath,fragmentShaderFilePath,vertexShader,fragmentShader);
	}
}