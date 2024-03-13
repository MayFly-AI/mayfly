#pragma once
#include <string>
struct Tensor;
class Dict;

class FrameDecoder {
public:
	virtual ~FrameDecoder(){}
	virtual bool DecodeFrame(Tensor* decoded, const uint8_t* data,int size)=0;
	virtual int GetTimers(int* timers, int maxTimers)const=0;
	virtual void Begin(const Dict* cfg)=0;
	virtual void End()=0;
	virtual void Flush()=0;
};


typedef FrameDecoder*(*FrameDecoderCreateFunc)(const Dict* cfg);
bool RegisterFrameDecoder(const std::string& encoding, FrameDecoderCreateFunc createFunc);

FrameDecoder* CreateFrameDecoder(const std::string& encoding, const Dict* decoderConfig);
void DestroyFrameDecoder(FrameDecoder* d);
