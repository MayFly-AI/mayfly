#include "decoder.h"
#include <unordered_map>
#include "shared/misc.h"
#include "shared/dict.h"
#include "shared/types.h"

int H264Decoder::GetTimers(int* timers, int maxTimers) const {
	if(!timers) {
		return countof(m_timers);
	}
	int n=std::min(countof(m_timers),maxTimers);
	for(int i=0;i<n;++i) {
		timers[i]=m_timers[i];
	}
	return n;
}

class H264DecoderFactory {
public:
	typedef H264Decoder* (*CreateDecoderFunc)();
	typedef std::unordered_map<std::string,CreateDecoderFunc> CreateMap;
	static bool Register(const char* name,CreateDecoderFunc func) {
		uprintf("Registering h264 decoder: %s\n",name);
		auto& reg=Get();
		auto str=std::string(name);
		auto found=reg.find(str);
		if(found!=reg.end()) {
			FATAL("A decoder with name \"%s\" is already registered",name);
			return false;
		}
		reg[str]=func;
		return true;
	}
	static H264Decoder* Create(const char* name) {
		uprintf("Creating decoder: %s\n",name);
		auto& reg=Get();
		auto str=std::string(name);
		auto found=reg.find(str);
		if(found==reg.end()) return nullptr;
		return found->second();
	}
	static CreateMap& Get() {
		static CreateMap reg;
		return reg;
	}
};

bool RegisterH264Decoder(const char* name,CreateH264DecoderFunc func){
	return H264DecoderFactory::Register(name,func);
}

H264Decoder* CreateH264Decoder(const Dict* dict) {
	std::string decoderType="openh264";
	if(dict) {
		dict->Get("type",&decoderType,decoderType);
	}
	H264Decoder* decoder=H264DecoderFactory::Create(decoderType.c_str());
	if(!decoder) {
		FATAL("No decoder named: %s\n",decoderType.c_str());
	}
	decoder->Begin(dict);
	return decoder;
}

void DestroyH264Decoder(H264Decoder* d) {
	d->End();
	delete d;
}

namespace {
	bool reg_h264=RegisterFrameDecoder("h264", [](const Dict* cfg)->FrameDecoder* {return CreateH264Decoder(cfg);});
}
