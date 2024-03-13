#ifdef _WIN32
#include <windows.h>
#include <malloc.h>
#endif

#if CUDA
#include "shared/misc.h"
#include <cuda.h>

#define CC(x) do{int _code=(x);if(_code)printf("CUDA FAILURE file:%s, line:%d, code:%d\n",__FILE__,__LINE__,_code);}while(0)

static_assert(sizeof(CUdeviceptr) == sizeof(void*));

struct CudaAllocator {
	CudaAllocator() {
		int iGpu=0;
		CUdevice dev=0;
		CC(cuInit(0));
		CC(cuDeviceGet(&dev,iGpu));
		CC(cuCtxCreate(&m_ctx,0,dev));
	}
	void* alloc(int size) {
		CUdeviceptr ptr;
		CC(cuCtxPushCurrent(m_ctx));
		CC(cuMemAlloc(&ptr,size));
		CC(cuCtxPopCurrent(0));
		void* ret=reinterpret_cast<void*>(ptr);
		return ret;
	}
	void free(void* p) {
		CC(cuCtxPushCurrent(m_ctx));
		CC(cuMemFree(reinterpret_cast<CUdeviceptr>(p)));
		CC(cuCtxPopCurrent(0));
	}
	static CudaAllocator& Get() {
		static CudaAllocator s_alloc;
		return s_alloc;
	}
	CUcontext m_ctx;
};

void* CudaAlloc(int size){
	return CudaAllocator::Get().alloc(size);
}
void CudaFree(void*p){
	CudaAllocator::Get().free(p);
}

#else
void* CudaAlloc(int size){return 0;}
void CudaFree(void* p){}
#endif
