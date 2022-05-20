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
	void RendererMain::rdDrawPrepare(){
		using namespace std;
		using namespace Render::Constant;
		//Get Shaders
		static string vertexShaderSource, fragmentShaderSource;
		GraphicsInterfaceUtility::getShadersDefault(&vertexShaderSource,&fragmentShaderSource);
		i32 success;
		char compilationInfo[shaderCompilationLogLen];
		auto vertexShader = glCreateShader(GL_VERTEX_SHADER);
		auto fragShader = glCreateShader(GL_FRAGMENT_SHADER);
		const char* vertexShaderSourceCs = vertexShaderSource.c_str();
		const char* fragShaderSourceCs = fragmentShaderSource.c_str();
		glShaderSource(vertexShader,1, &vertexShaderSourceCs ,NULL);
		glShaderSource(fragShader,1,&fragShaderSourceCs,NULL);
		//Compile Vertex Shader
		glCompileShader(vertexShader);
		glGetShaderiv(vertexShader,GL_COMPILE_STATUS,&success);
		if(!success){
			glGetShaderInfoLog(vertexShader,shaderCompilationLogLen,NULL,compilationInfo);
			dbg_toutput("Vertex Shader Compilation Failed");
			dbg_toutput(compilationInfo);
			dbg_toutput("Vertex Shader Info");
			dbg_toutput(vertexShaderSourceCs);
			pr_assert(false);
		}else{
			dbg_toutput("Vertex Shader Compilation Done");
		}
		//Compile Fragment Shader
		glCompileShader(fragShader);
		glGetShaderiv(fragShader,GL_COMPILE_STATUS,&success);
		if(!success){
			glGetShaderInfoLog(fragShader,shaderCompilationLogLen,NULL,compilationInfo);
			dbg_toutput("Fragment Shader Compilation Failed");
			dbg_toutput(compilationInfo);
			pr_assert(false);
		}else{
			dbg_toutput("Fragment Shader Compilation Done");
		}
		//Create Program
		auto shaderProgram = glCreateProgram();

	}
	void RendererMain::rdTest() {
		using namespace Render::Constant;
		using namespace std;
		std::cout << "Hello World" <<endl;
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
		
		rdDrawPrepare();
		//Start Drawing !
		while(!glfwWindowShouldClose(window)){
			rdRender();
			glfwSwapBuffers(window);
			glfwPollEvents();    
		}
	}
}