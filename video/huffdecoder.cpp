#include "framedecoder.h"
#include "huffdecoder.h"
#include "shared/dict.h"
#include "shared/misc.h"
#include "memory/tensor.h"
#include "shared/dctquant16.h"

class HuffDecoder : public FrameDecoder {
public:
	bool DecodeFrame(Tensor* decoded,const uint8_t*data,int size){
		std::vector<uint8_t> frame_uint8;
		frame_uint8.assign(data, data+size);
		int width, height;
		std::vector<uint16_t> decoded2;
		if(!m_decoder.DecodeQuant16(&decoded2,&width,&height,frame_uint8)) {
			FATAL("DecodeQuant16 failed\n");
			return false;
		}
		int shape[] = {height,width};
		Tensor depth = CreateCPUTensor(2,shape,sizeof(float));
		float* ptr = (float*)depth.m_data;
		for(size_t i=0;i<decoded2.size();++i) {
			ptr[i] = decoded2[i] * 4.0f/4095.0f;
		}
		*decoded=std::move(depth);
		return true;
	}
	virtual int GetTimers(int* timers, int maxTimers)const{
		return 0;
	}
	void End(){}
	void Begin(const Dict* cfg){}
	void Flush(){}

	DecoderDCT16 m_decoder;
};

FrameDecoder* CreateHuffDecoder(const Dict* cfg) {
	auto* decoder=new HuffDecoder;
	decoder->Begin(cfg);
	return decoder;
}

namespace {
	bool reg_huff=RegisterFrameDecoder("huff", [](const Dict* cfg)->FrameDecoder* {return CreateHuffDecoder(cfg);});
}

int huff_static_link_use=0;
