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
	
	return 0;
}