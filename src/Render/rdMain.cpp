#include "../../include/Render/rdMain.h"
#include "../../include/Render/rdDefs.h"
#include "../../include/Common/cmTypesDefsGraphics.h"
namespace Render {
	RendererMain* RendererMain::inst = nullptr;
	RendererMain* RendererMain::rdGetInstance() {
		if (RendererMain::inst == nullptr) {
			RendererMain::inst = new RendererMain();
		}
		return RendererMain::inst;
	}
	void RendererMain::rdRender(){
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		if(enableDraw){
			glDrawElements(GL_TRIANGLES, aDrawTriangles, GL_UNSIGNED_INT, 0);
		}
		
	}
	void RendererMain::rdSetRenderMesh(Common::Mesh::ShaderCompatibleMeshData* meshData){
		u32 VBO;
		glGenBuffers(1, &VBO);

		u32 EBO;
		glGenBuffers(1, &EBO);

		u32 VAO;
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(f32)*meshData->vertexDataLen, meshData->vertexData, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(u32)*meshData->faceDataLen, meshData->faceData, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(f32), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(f32), (void*)(3 * sizeof(f32)));
		glEnableVertexAttribArray(1);

		glUseProgram(aShaderProgramID);
		glBindVertexArray(VAO);
		aDrawTriangles = meshData->vertexDataLen;
		enableDraw = true;
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
		glAttachShader(shaderProgram, vertexShader);
		glAttachShader(shaderProgram, fragShader);
		glLinkProgram(shaderProgram);
		glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
		if(!success){
			glGetProgramInfoLog(shaderProgram,shaderCompilationLogLen,NULL,compilationInfo);
			dbg_toutput("Shader Program Linking Failed");
			dbg_toutput(compilationInfo);
			pr_assert(false);
		}else{
			dbg_toutput("Shader Program Linking Done");
		}
		aShaderProgramID = shaderProgram;

		//Transform
		Common::Camera::CoordinateSystems csMats;
		csMats.modelMatrix =  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
		csMats.projectionMatrix = glm::mat4(1.0f);
		csMats.projectionMatrix = glm::perspective(45.0f, 1.0f*viewportWidth/viewportHeight , 1.0f, 1000.0f);
		csMats.viewMatrix = glm::mat4(1.0f);

		glm_printmat4(csMats.projectionMatrix);

		//Use Shader
		glUseProgram(shaderProgram);
		GraphicsInterfaceUtility::applyTransform(shaderProgram,&csMats);
		
		//Shader
		glDeleteShader(vertexShader);
		glDeleteShader(fragShader);
		
	}
	void RendererMain::rdRenderStart(){
		//Start Drawing !
		while(!glfwWindowShouldClose(aWindow)){
			rdInputProcessing();
			rdUpdateViewMatrix();
			rdRender();
			glfwSwapBuffers(aWindow);
			glfwPollEvents();    
		}
	}
	void RendererMain::rdInputProcessing(){
		if(glfwGetKey(aWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        	glfwSetWindowShouldClose(aWindow, true);
		float cameraSpeed = 0.05f; // adjust accordingly
		if (glfwGetKey(aWindow, GLFW_KEY_W) == GLFW_PRESS)
			observer->camPos += cameraSpeed * observer->camFront;
		if (glfwGetKey(aWindow, GLFW_KEY_S) == GLFW_PRESS)
			observer->camPos -= cameraSpeed * observer->camFront;
		if (glfwGetKey(aWindow, GLFW_KEY_A) == GLFW_PRESS)
			observer->camPos -= glm::normalize(glm::cross(observer->camFront, observer->camUp)) * cameraSpeed;
		if (glfwGetKey(aWindow, GLFW_KEY_D) == GLFW_PRESS)
			observer->camPos += glm::normalize(glm::cross(observer->camFront, observer->camUp)) * cameraSpeed;
	}
	void RendererMain::rdUpdateViewMatrix(){
		using namespace Render::Constant;
		Common::Camera::CoordinateSystems csMats;
		csMats.modelMatrix =  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
		csMats.projectionMatrix = glm::perspective(glm::radians(aFov), 1.0f*viewportWidth/viewportHeight , 1.0f, 1000.0f);
		glm::vec3 camDirection = glm::normalize(observer->camPos - observer->camLookAt);
		glm::vec3 camRight = glm::normalize(glm::cross(observer->camUp, camDirection));
		glm::vec3 camUp = glm::cross(camDirection, camRight);
		csMats.viewMatrix = glm::lookAt(observer->camPos,observer->camFront + observer->camPos,observer->camUp);
		GraphicsInterfaceUtility::applyTransform(aShaderProgramID,&csMats);
		
	}
	void RendererMain::rdMouseCallback(GLFWwindow* window, double xpos, double ypos){
		auto _this = RendererMain::rdGetInstance();
		if(!_this->mouseRecorder->initialized) {
			_this->mouseRecorder->lastX = xpos;
			_this->mouseRecorder->lastY = ypos;
			_this->mouseRecorder->initialized = true;
		}
		
		float xoffset = xpos - _this->mouseRecorder->lastX;
		float yoffset = _this->mouseRecorder->lastY - ypos;
		_this->mouseRecorder->lastX = xpos;
		_this->mouseRecorder->lastY = ypos;

		float sensitivity = 0.1f;
		xoffset *= sensitivity;
		yoffset *= sensitivity;

		_this->mouseRecorder->yaw += xoffset;
		_this->mouseRecorder->pitch += yoffset;

		if(_this->mouseRecorder->pitch > 89.0f)
			_this->mouseRecorder->pitch =  89.0f;
		if(_this->mouseRecorder->pitch < -89.0f)
			_this->mouseRecorder->pitch = -89.0f;
		glm::vec3 front;
		front.x = cos(glm::radians(_this->mouseRecorder->pitch)) * cos(glm::radians(_this->mouseRecorder->yaw));
		front.y = sin(glm::radians(_this->mouseRecorder->pitch));
		front.z = cos(glm::radians(_this->mouseRecorder->pitch)) * sin(glm::radians(_this->mouseRecorder->yaw));
		_this->observer->camFront = front;
	}
	void RendererMain::rdScrollCallback(GLFWwindow* window, double xoffset, double yoffset){
		using namespace Render::Constant;
		auto _this = RendererMain::rdGetInstance();
		if(_this->aFov >= 1.0f && _this->aFov <= 45.0f)
			_this->aFov -= yoffset;
		if(_this->aFov <= 1.0f)
			_this->aFov = 1.0f;
		if(_this->aFov >= 45.0f)
			_this->aFov = 45.0f;
	}
	void RendererMain::rdTest() {
		using namespace Render::Constant;
		using namespace std;
		observer = new Common::Camera::Camera();
		observer->camPos=glm::vec3(0,0,1);
		observer->camLookAt=glm::vec3(0,0,0);
		mouseRecorder = new Render::Interaction::MouseRecorder();
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
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		glfwSetCursorPosCallback(window, rdMouseCallback);
		glfwSetScrollCallback(window,rdScrollCallback);
		aWindow = window;
		glEnable(GL_DEPTH_TEST);
	}
}