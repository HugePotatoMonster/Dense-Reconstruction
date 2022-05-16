#include "./include/Test/tsTest.h"
#include "./include/DepthEstimation/deDepthEstimationHelper.h"
#include "./include/Misc/msAuxiliaryUtils.h"
#include "./include/Parallel/CUDA/ccuDeclarations.h"
using namespace std;

int main() {
	Test::Test* test = new Test::Test();
	test->binocularSGM();
	
	return 0;
}