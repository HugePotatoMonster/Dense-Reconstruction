#include "./include/Test/tsTest.h"
#include "./include/DepthEstimation/deDepthEstimationHelper.h"
#include "./include/Misc/msAuxiliaryUtils.h"
#include "./include/Parallel/CUDA/ccuDeclarations.h"
#include "./include/Render/rdMain.h"
#include "tsdf.cpp"
using namespace std;

int main() {
	//
	Test::Test* test = new Test::Test();
	test->testj();

	/*
	using namespace std;
	auto p = Render::RendererMain::rdGetInstance();
	Common::Mesh::Mesh* mesh = new Common::Mesh::Mesh();
	Common::Mesh::ShaderCompatibleMeshData meshData;
	generateTSDF(mesh);
	cout<<"MESHV="<<mesh->v.size();
	Render::GraphicsInterfaceUtility::convertMeshToArray(mesh,&meshData);
	p->rdTest();
	p->rdDrawPrepare();
	p->rdSetRenderMesh(&meshData);
	p->rdRenderStart();*/
	
	return 0;
}