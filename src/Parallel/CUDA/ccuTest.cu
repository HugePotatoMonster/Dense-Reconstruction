#include "../../../include/Parallel/CUDA/ccuDeclarations.h"

#ifdef PR_CUDA_ENABLE

namespace Parallel {
	namespace CUDA {
		namespace Test {
			cu_global void helloworld() {
				printf("HelloWorld");
			}
			void helloworldCall() {
				helloworld <<<1, 2>>> ();
				cu_sync();
			}
		}
		
	}
}

#endif