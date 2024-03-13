#pragma once
#include <vector>

struct Tensor{
	Tensor()=default;
	Tensor&operator=(Tensor&&other)noexcept;
	Tensor(Tensor&&other)noexcept;
	~Tensor();

	// delete regular copy and and assign
	Tensor(const Tensor&)=delete;
	Tensor&operator=(Tensor&other)=delete;

	int ByteSize() const;
	void Free();

	enum AllocatorType{ALLOC_CPU=0,ALLOC_CUDA=1};
	enum{MAX_DIMENSIONS=4};
	int m_shape[MAX_DIMENSIONS];
	int m_stride[MAX_DIMENSIONS]; // stride is counting number of elements - not bytes
	int m_dimensions{0};
	int m_elementSize{0};
	AllocatorType m_type{};
	void*m_data{nullptr};

	typedef void(*TFreeMemoryFunc)(void*);
	TFreeMemoryFunc m_free{nullptr};
};

Tensor CreateCUDATensor(int dims, const int* shape, int elementSize);
Tensor CreateCPUTensor(int dims, const int* shape, int elementSize);
