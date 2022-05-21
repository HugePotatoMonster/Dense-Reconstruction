#pragma once
#include "../Common/cmTypeDefs.h"
#include "../Common/cmTypesDefsGraphics.h"
#include "rdDefs.h"
#include "./rdUtility.h"
namespace Render {
	class RendererMain {
	private:
		static RendererMain* inst;
		Common::Camera::Camera* observer;
		u32 aShaderProgramID = 0;
		u32 aDrawTriangles = 0;
		u32 enableDraw = false;
		f32 aFov = 45.0f;
		GLFWwindow* aWindow = nullptr;
		Render::Interaction::MouseRecorder* mouseRecorder;
		//Singleton
		RendererMain(){}
	public:
		static RendererMain* rdGetInstance();
		void rdTest();
		void rdRender();
		void rdRenderStart();
		void rdDrawPrepare();
		void rdSetRenderMesh(Common::Mesh::ShaderCompatibleMeshData* meshData);
		void rdUpdateViewMatrix();
		void rdInputProcessing();
		static void rdMouseCallback(GLFWwindow* window, double xpos, double ypos);
		static void rdScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
	};
}