#pragma once
#include "framedecoder.h"

class H264Decoder : public FrameDecoder {
public:
	virtual int Channels()const=0;		//RGB=3 RGBA=4
	virtual int GetTimers(int* timers, int maxTimers) const;
	int m_timers[2]={0,0};		//0=decodeh264 1=convertYUV
};

typedef H264Decoder*(*CreateH264DecoderFunc)();
bool RegisterH264Decoder(const char* name, CreateH264DecoderFunc func);

H264Decoder* CreateH264Decoder(const Dict* dict);
void DestroyH264Decoder(H264Decoder* d);
