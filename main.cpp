// #include "./include/Test/tsTest.h"
// #include "./include/DepthEstimation/deDepthEstimationHelper.h"
// #include "./include/Misc/msAuxiliaryUtils.h"
// #include "./include/Parallel/CUDA/ccuDeclarations.h"
// #include "./include/Render/rdMain.h"
// #include "./include/RealtimeShow/RealtimeHelper.h"
#include "tsdf.cpp"

using namespace std;

int main() {

    SurfaceMesh* mesh = new SurfaceMesh;
	
	Viewer viewer("Result");

	thread gen(generateTSDF, &viewer, mesh);
	gen.detach();

	while(true){
		cout << "wait" << endl;
		if(!meshFlag){
			viewer.run();
			return 0;
		}
	};
	//
	// Test::Test* test = new Test::Test();
	// test->testj();

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