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
	void GraphicsInterfaceUtility::applyTransform(u32 shaderProgram,Common::Camera::CoordinateSystems* matrices){
		unsigned int transformLoc1 = glGetUniformLocation(shaderProgram, "modelMat");
		glUniformMatrix4fv(transformLoc1, 1, GL_FALSE, glm::value_ptr(matrices->modelMatrix));
		unsigned int transformLoc2 = glGetUniformLocation(shaderProgram, "viewMat");
		glUniformMatrix4fv(transformLoc2, 1, GL_FALSE, glm::value_ptr(matrices->viewMatrix));
		unsigned int transformLoc3 = glGetUniformLocation(shaderProgram, "projectionMat");
		glUniformMatrix4fv(transformLoc3, 1, GL_FALSE, glm::value_ptr(matrices->projectionMatrix));
	}
	void GraphicsInterfaceUtility::convertMeshToArray(Common::Mesh::Mesh* inMesh,OUT_ARG Common::Mesh::ShaderCompatibleMeshData* outMesh){
		using namespace std;
		//Allocate Memory
		u32 vertexSize = (u32)(inMesh->v.size()) * 7;
		u32 faceSize = 0;
		for(i32 i=0;i<inMesh->f.size();i++){
			faceSize += (inMesh->f[i].size() - 2)*3;
		}
		outMesh->faceData = new u32[faceSize];
		outMesh->vertexData = new f32[vertexSize];
		outMesh->faceDataLen = faceSize;
		outMesh->vertexDataLen = vertexSize;
		f64 avgX = 0,avgY=0,avgZ=0;
		//Fill in Vertex Array
		for(u32 i=0;i<inMesh->v.size();i++){
			outMesh->vertexData[i*7] = inMesh->v[i].x;
			outMesh->vertexData[i*7+1] = inMesh->v[i].y;
			outMesh->vertexData[i*7+2] = -inMesh->v[i].z;
			outMesh->vertexData[i*7+3] = inMesh->c[i].x;
			outMesh->vertexData[i*7+4] = inMesh->c[i].y;
			outMesh->vertexData[i*7+5] = inMesh->c[i].z;
			outMesh->vertexData[i*7+6] = 1.0f;
			avgX += inMesh->v[i].x;
			avgY += inMesh->v[i].y;
			avgZ += inMesh->v[i].z;
		}
		cout<<"TOTAL VS"<<inMesh->v.size();
		avgX /= inMesh->v.size();
		avgY /= inMesh->v.size();
		avgZ /= inMesh->v.size();
		cout<<"AVG:"<<avgX<<" "<<avgY<<" "<<avgZ<<endl;
		//Fill in Face Array, Assume all polygons are convex
		u32 cur = 0;
		for(u32 i=0;i<inMesh->f.size();i++){
			for(u32 j=2;j<inMesh->f[i].size();j++){
				outMesh->faceData[cur] = inMesh->f[i][0];
				outMesh->faceData[cur+1] = inMesh->f[i][j-1];
				outMesh->faceData[cur+2] = inMesh->f[i][j];
				cur+=3;
			}
		}
	}
}