#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <numeric>
#include <map>
#include <vector>
#include <memory> 

#include <stdio.h>
#include <sys/types.h>

#include <string>

#include "shared/types.h"
#include "shared/misc.h"
#include "shared/std_ext.h"

#include "fecbuffer.h"
#include "fec.h"

class FecBufferImpl : public FecBuffer, public FEC {
	public:
		FecBufferImpl(int _k,int _n);
		virtual ~FecBufferImpl();
		virtual void Encode(std::vector<std::vector<uint8_t>>* encoded,const void* data,int dataBytesize,int blockSize)const;
		virtual bool Decode(void* data,int dataBytesize,std::vector<bool>& received,const std::vector<std::vector<uint8_t>>& encoded,int blockSize)const;
		virtual uint8_t GetK()const{return k;};
		virtual uint8_t GetN()const{return n;};
};

FecBufferImpl::FecBufferImpl(int _k,int _n) : FEC(_k,_n){
}
FecBufferImpl::~FecBufferImpl() {
}

bool FecBufferImpl::Decode(void* data,int dataBytesize,std::vector<bool>& received,const std::vector<std::vector<uint8_t>>& encoded,int blockSize)const {
	std::vector<uint8_t> decoding_matrix(k*k);
	std::vector<int> indexes(k);
	std::vector<const uint8_t*> shares_begins(k);

#if 1
	int indices[64];
	const uint8_t* blocks[64];
	int ix=0;
	for(int i=0;i!=(int)encoded.size();i++) {
		if(received[i]) {
			blocks[ix]=encoded.at(i).data();
			indices[ix++]=i;
		}
	}
	if(ix<k)
		exit(0);		//Cannot decode
	int ixf=0;
	int ixb=ix-1;
	bool missing_primary_share=false;
	for(int i=0;i<k;i++) {
		int share_id=0;
		int shares_b_num=indices[ixf];
		if(shares_b_num==i) {
			share_id=shares_b_num;
			shares_begins[i]=blocks[ixf];
			ixf++;
		}else{
			share_id=indices[ixb];
			shares_begins[i]=blocks[ixb];
			ixb--;
			missing_primary_share=true;
		}
		if(share_id<0 || share_id>=n) {
			exit(0);
		}
		if(share_id<k) {
			decoding_matrix[i*(k+1)]=1;
		}else{
			memcpy(&decoding_matrix[i*k],&enc_matrix[share_id*k],k);
		}
		indexes[i]=share_id;
	}
#else
	std::map<int,std::vector<uint8_t>> shares;
	for(int i=0;i!=(int)encoded.size();i++) {
		if(received[i])
			shares.insert(std::make_pair(i,encoded[i]));
	}
	auto shares_b_iter=std::begin(shares);
	auto shares_e_iter=std::rbegin(shares);
	bool missing_primary_share=false;
	for(int i=0; i < k; i++) {
		int share_id=0;
		int shares_b_num=shares_b_iter->first;
		if(shares_b_num==i) {
			share_id=shares_b_num;
			auto& data=shares_b_iter->second;
			shares_begins[i]=&*std::begin(data);
			++shares_b_iter;
		} else {
			share_id=shares_e_iter->first;
			auto& data=shares_e_iter->second;
			shares_begins[i]=&*std::begin(data);
			++shares_e_iter;
			missing_primary_share=true;
		}
		if(share_id<0 || share_id>=n) {
			exit(0);
		}
		if(share_id < k) {
			decoding_matrix[i*(k+1)]=1;
		} else {
			memcpy(&decoding_matrix[i*k],&enc_matrix[share_id*k],k);
		}
		indexes[i]=share_id;
	}
#endif
	// shortcut: if we have all the original data shares,we don't need to
	// perform a matrix inversion or do anything else.
	if(!missing_primary_share) {
		for(int i=0;i<int(indexes.size());++i) {
			memcpy((uint8_t*)data+i*blockSize,shares_begins[i],blockSize);
		}
		return true;
	}
	invertMatrix(decoding_matrix,k);
	std::vector<uint8_t> buf(blockSize);
	for(int i=0; i < int(indexes.size()); ++i) {
		if(indexes[i]>=k) {
			std::fill(buf.begin(),buf.end(),0);
			for(int col=0; col < k; ++col) {
				addmul(buf.data(),buf.data()+blockSize,shares_begins[col],decoding_matrix[i*k+col]);
			}
			memcpy((uint8_t*)data+i*blockSize,buf.data(),blockSize);
		}else{
			memcpy((uint8_t*)data+i*blockSize,shares_begins[i],blockSize);
		}
	}
	return true;
}

void FecBufferImpl::Encode(std::vector<std::vector<uint8_t>>* encoded,const void* data1,int dataBytesize1,int blockSize)const {
	//uint64_t t0=GetTimeMicroseconds();
	int dataBytesize=k*blockSize;
	if(dataBytesize1>dataBytesize)
		FATAL("WTF");
	uint8_t* src=(uint8_t*)data1;
	encoded->resize(n);
	int remain=dataBytesize1;
	for(int i=0;i<k;i++) {
		encoded->at(i).resize(blockSize);
		uint8_t* dst=encoded->at(i).data();
		if(remain>blockSize) {
			memcpy(dst,src+i*blockSize,blockSize);
		}else{
			if(remain<=0) {
				memset(dst,0,blockSize);
			}else{
				memcpy(dst,src+i*blockSize,remain);
				memset(dst+remain,0,blockSize-remain);
			}
		}
		remain-=blockSize;
	}
	for(int i=k;i<n;i++) {
		encoded->at(i).resize(blockSize);
		uint8_t* dst=encoded->at(i).data();
		memset(dst,0,blockSize);
		for(int j=0; j < k; j++) {
			addmul(dst,dst+blockSize,encoded->at(j).data(),enc_matrix[i*k+j]);
		}
	}
	//uint64_t t1=GetTimeMicroseconds();
	//uprintf("tst %dus\n",(int)(t1-t0));

/*
//Validate decode
{
	uint8_t* data=new uint8_t[dataBytesize];
	memset(data,0,dataBytesize);
	memcpy(data,data1,dataBytesize1);
	std::vector<uint8_t> dataIn(k*blockSize);
	memcpy(dataIn.data(),data,k*blockSize);

	std::vector<bool> received={false,true,true,true,true,false,false,false,true,true,true,true};
	std::vector<uint8_t> dataOut(k*blockSize);
	Decode(dataOut.data(),(int)dataOut.size(),received,*encoded,blockSize);
	if(dataIn!=dataOut) {
		FATAL("not same %d!=%d\n",dataIn.size(),dataOut.size());
	}else{
		//uprintf("Validate Ok\n");
	}
	delete [] data;
}
*/
}
//};

int test() {
	int k=8;
	int n=12;
	int blockSize=1400;
	std::vector<uint8_t> data(k*blockSize);
	for(int i=0;i!=(int)data.size();i++)
		data[i]=i&0xff;
	std::vector<std::vector<uint8_t>> encoded;
	FecBufferImpl ft(k,n);
	ft.Encode(&encoded,data.data(),(int)data.size(),blockSize);
	std::vector<uint8_t> data1(k*blockSize);
	//std::vector<bool> received={true,true,true,true,true,true,true,true,false,false,false,false};
	std::vector<bool> received={false,true,true,true,true,false,false,false,true,true,true,true};
	//std::vector<bool> received={false,true,true,true,true,true,true,true,false,false,false,true};
	ft.Decode(data1.data(),(int)data1.size(),received,encoded,blockSize);
	if(data!=data1) {
		::printf("encode->decode mismatch\n");
	}
	return 0;
}

FecBuffer* CreateFecBuffer(int k,int n) {
	return new FecBufferImpl(k,n);
}
void DestroyFecBuffer(FecBuffer* fec) {
	delete fec;
}

