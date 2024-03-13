#include <string>
#include "video/decoder.h"
#include "memory/tensor.h"
#include <cstring>

class NullDecodeImpl : public H264Decoder{
public:
	virtual ~NullDecodeImpl(){}
	virtual void Begin(const Dict* config){}
	virtual bool DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264);
	virtual void End(){};
	virtual void Flush(){};
	virtual int Channels()const{return 1;}

	static H264Decoder* Create() { return new NullDecodeImpl(); }
private:
	static const bool s_registered;
};
const bool NullDecodeImpl::s_registered=RegisterH264Decoder("nodecode", NullDecodeImpl::Create);

bool NullDecodeImpl::DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264) {
	*decoded=CreateCPUTensor(1,&bytesizeH264,sizeof(uint8_t));
	memcpy(decoded->m_data,packetH264,bytesizeH264);
	return true;
}

int nodecode_static_link_use=0;
