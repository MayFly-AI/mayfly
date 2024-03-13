#include "tensor.h"
#include <cstring>
#include <cstdlib>
#include "cuda_alloc.h"
#include "shared/misc.h"

#define ENABLE_TENSOR_POINTER_TRACKING 1

#ifdef ENABLE_TENSOR_POINTER_TRACKING
#include "pointer_tracking.h"
static PointerTracking g_CPU_Tracking("CPU");
static PointerTracking g_CUDA_Tracking("CUDA");
#endif

Tensor& Tensor::operator=(Tensor&&other)noexcept{
	if(this!=&other){
		ASSERT(!m_data, "Tensor leak\n");
		memcpy(this,&other,sizeof(other));
		other.m_data=nullptr;
	}
	return*this;
}
Tensor::Tensor(Tensor&&other)noexcept{
	ASSERT(!m_data, "Tensor leak\n");
	memcpy(this,&other,sizeof(other));
	other.m_data=nullptr;
}
Tensor::~Tensor() {
	Free();
}
void Tensor::Free() {
	if(m_data){
		m_free(m_data);
	}
	memset(this, 0, sizeof(*this));
}
int Tensor::ByteSize() const {
	int count=1;
	for(int i=0;i<m_dimensions;++i)
		count*=m_shape[i];
	return count*m_elementSize;
}
void SetDimensions(Tensor* res, int dims, const int* shape, int elementSize) {
	ASSERT(dims <= Tensor::MAX_DIMENSIONS, "Too many dimensions: %d\n", dims);
	memset(res->m_shape, 0, sizeof(res->m_shape));
	memset(res->m_stride, 0, sizeof(res->m_stride));
	memcpy(res->m_shape, shape, sizeof(int)*dims);
	int s=1;
	for(int i=dims-1;i>=0;--i) {
		res->m_stride[i] = s;
		s*=shape[i];
	}
	res->m_dimensions=dims;
	res->m_elementSize=elementSize;
}

inline void* CudaAllocTrack(int size) {
	void* p=CudaAlloc(size);
#if ENABLE_TENSOR_POINTER_TRACKING
	g_CUDA_Tracking.Register(p);
#endif
	return p;
}
inline void CudaFreeTrack(void* p) {
#if ENABLE_TENSOR_POINTER_TRACKING
	g_CUDA_Tracking.Unregister(p);
#endif
	CudaFree(p);
}

Tensor CreateCUDATensor(int dims, const int* shape, int elementSize) {
	Tensor res;
	SetDimensions(&res, dims, shape, elementSize);
	res.m_type=Tensor::AllocatorType::ALLOC_CUDA;
	res.m_data=CudaAllocTrack(res.ByteSize());
	res.m_free=CudaFreeTrack;
	return res;
}

static void* AllocCPUTensor(int byteSize){
	void*p=malloc(byteSize);
	ASSERT(p,"Out of memory");
#if ENABLE_TENSOR_POINTER_TRACKING
	g_CPU_Tracking.Register(p);
#endif
	return p;
}
static void FreeCPUTensor(void* p) {
#if ENABLE_TENSOR_POINTER_TRACKING
	g_CPU_Tracking.Unregister(p);
#endif
	free(p);
}

Tensor CreateCPUTensor(int dims, const int* shape, int elementSize) {
	Tensor res;
	SetDimensions(&res, dims, shape, elementSize);
	res.m_type=Tensor::AllocatorType::ALLOC_CPU;
	res.m_data=AllocCPUTensor(res.ByteSize());
	res.m_free=FreeCPUTensor;
	return res;
}

void TestTensor() {
	int dims[] = {4,8};
	{
		Tensor t = CreateCPUTensor(2, dims, sizeof(float));
		Tensor t2 = std::move(t);
	}

	std::vector<Tensor> v;
	v.reserve(100);
	int shape[4] = {100,32,5,7};
	for(int i=0;i<100;++i) {
		v.push_back(CreateCUDATensor(4, shape, sizeof(float)));
	}
	printf("clearing tensor  vector\n");
	v.clear();

#if ENABLE_TENSOR_POINTER_TRACKING
	ASSERT(g_CPU_Tracking.Size() == 0, "expected no CPU allocations");
#endif

	printf("TestTensor done\n");
	exit(0);
}
