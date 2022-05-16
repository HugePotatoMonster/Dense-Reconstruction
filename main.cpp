#include "./include/Test/tsTest.h"
#include "./include/DepthEstimation/deDepthEstimationHelper.h"
#include "./include/Misc/msAuxiliaryUtils.h"
#include "./include/Parallel/CUDA/ccuDeclarations.h"
#include "tsdf.cpp"
using namespace std;

int main() {
	//generateTSDF();
	Test::Test* test = new Test::Test();
	test->monocularMotionSGMDepth();
	
	return 0;
}