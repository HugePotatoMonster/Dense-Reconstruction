#pragma once
#include "../Common/cmTypeDefs.h"
#include "./rdUtility.h"
namespace Render {
	class RendererMain {
	private:
		static RendererMain* inst;
		RendererMain(){}
	public:
		static RendererMain* rdGetInstance();
		void rdTest();
		void rdRender();
		void rdDrawPrepare();
	};
}