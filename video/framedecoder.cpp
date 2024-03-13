#include "framedecoder.h"
#include <unordered_map>
#include "shared/misc.h"

class FrameDecoderFactory {
public:
	typedef std::unordered_map<std::string,FrameDecoderCreateFunc> CreateMap;
	static FrameDecoder* Create(const std::string& encoding, const Dict* decoderConfig) {
		auto& createFuncs=Get();
		auto found=createFuncs.find(encoding);
		if(found==createFuncs.end()) {
			return nullptr;
		}
		return found->second(decoderConfig);
	}
	static bool RegisterFrameDecoder(const std::string& encoding, FrameDecoderCreateFunc createFunc) {
		uprintf("Registering decoder %s\n",encoding.c_str());
		auto& createFuncs=Get();
		createFuncs[encoding]=createFunc;
		return true;
	}
	static CreateMap& Get() {
		static CreateMap s_createFuncs;
		return s_createFuncs;
	}
};

bool RegisterFrameDecoder(const std::string& encoding, FrameDecoderCreateFunc createFunc) {
	return FrameDecoderFactory::RegisterFrameDecoder(encoding, createFunc);
}

FrameDecoder* CreateFrameDecoder(const std::string& encoding, const Dict* decoderConfig) {
	return FrameDecoderFactory::Create(encoding, decoderConfig);
}

void DestroyFrameDecoder(FrameDecoder* d) {
	if(d) {
		d->End();
		delete d;
	}
}

#ifdef STATIC_LINK_DECODERS
// Workaround for linker removing unused statically linked object
// Disable this when used as dynamic library.
void WorkaroundStaticLinkDecoders() {
	extern int avdecode_static_link_use;
	avdecode_static_link_use=1;
	extern int nvdecode_static_link_use;
	nvdecode_static_link_use=1;
	extern int openh264_static_link_use;
	openh264_static_link_use=1;
	extern int plainh264_static_link_use;
	plainh264_static_link_use=1;
	extern int nodecode_static_link_use;
	nodecode_static_link_use=1;

	extern int huff_static_link_use;
	huff_static_link_use=1;
}
#endif
