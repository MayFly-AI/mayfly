#pragma once
#include <stdint.h>
#include <vector>

class FecBuffer {
	public:
		virtual ~FecBuffer(){};
		virtual uint8_t GetK()const=0;
		virtual uint8_t GetN()const=0;
		virtual void Encode(std::vector<std::vector<uint8_t>>* encoded,const void* data,int dataBytesize,int blockSize)const=0;
		virtual bool Decode(void* data,int dataBytesize,std::vector<bool>& received,const std::vector<std::vector<uint8_t>>& encoded,int blockSize)const=0;
};
FecBuffer* CreateFecBuffer(int k,int n);
void DestroyFecBuffer(FecBuffer*);

