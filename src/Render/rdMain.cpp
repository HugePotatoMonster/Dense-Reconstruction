#include "../../include/Render/rdMain.h"
#include "../../include/Render/rdDefs.h"
namespace Render {
	RendererMain* RendererMain::inst = nullptr;
	RendererMain* RendererMain::rdGetInstance() {
		if (RendererMain::inst == nullptr) {
			RendererMain::inst = new RendererMain();
		}
		return RendererMain::inst;
	}
	void RendererMain::rdRender(){
		glClearColor(1.0f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
	}
	void RendererMain::rdTest() {
		using namespace Render::Constant;
		std::cout << "Hello World";
		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		// For MacOS
		//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); 
		GLFWwindow* window = glfwCreateWindow(viewportWidth, viewportHeight, windowTitle, NULL, NULL);
		if (window == NULL){
			std::cout << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
		}
		glfwMakeContextCurrent(window);
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			std::cout << "Failed to initialize GLAD" << std::endl;
		}
		glViewport(0, 0, viewportWidth, viewportHeight);
		GraphicsInterfaceUtility::initialize(window);
		while(!glfwWindowShouldClose(window)){
			rdRender();
			glfwSwapBuffers(window);
			glfwPollEvents();    
		}
	}
}