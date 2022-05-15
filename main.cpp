#include "./include/Test/tsTest.h"
using namespace std;

int main() {
	//Test
	Test::Test* tests = new Test::Test();
	tests->monocularMotionSGMDepth();
	return 0;
}