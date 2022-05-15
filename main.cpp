#include "./include/Test/tsTest.h"
using namespace std;

int main() {
	//Test
	Test::Test* tests = new Test::Test();
	tests->monocularMotionSGM();
	return 0;
}