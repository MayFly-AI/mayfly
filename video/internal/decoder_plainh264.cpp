#include "video/decoder.h"
#include "shared/misc.h"
#include "shared/types.h"
#include "shared/math.h"
#include "shared/file.h"
#include "memory/tensor.h"
#include "3rdparty/plainh264/decodeimage.h"

class PlainH264Impl : public H264Decoder {
public:
	virtual ~PlainH264Impl(){}
	virtual void Begin(const Dict* config);
	virtual bool DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264);
	virtual void End();
	virtual void Flush();
	virtual int Channels()const{return 3;}
	void* m_decoder=0;
	unsigned long long m_ts=0;

	static H264Decoder* Create() { return new PlainH264Impl(); }
	static const bool s_registered;
};

const bool PlainH264Impl::s_registered=RegisterH264Decoder("plainh264", PlainH264Impl::Create);

#define YUV2B(Y,U,V) (int)(1.164f*(Y-16)+2.018f*(U-128))
#define YUV2G(Y,U,V) (int)(1.164f*(Y-16)-0.813f*(V-128)-0.391f*(U-128))
#define YUV2R(Y,U,V) (int)(1.164f*(Y-16)+1.596f*(V-128))

static void ConvertYUV(uint8_t* rgb,const uint8_t* ybuf,const uint8_t* ubuf,const uint8_t* vbuf,int s0,int s1,int width,int height,int channels) {
	int p0=0,p1=0,pd=0;
	for(int y=0;y<(height>>1);y++) {
		for(int i=0;i<2;++i) {
			for(int x=0;x<(width>>1);x++) {
				int u=ubuf[p1+x];
				int v=vbuf[p1+x];
				int y0=ybuf[p0+(x<<1)];
				int y1=ybuf[p0+(x<<1)+1];
				int b0=YUV2B(y0,u,v);
				int g0=YUV2G(y0,u,v);
				int r0=YUV2R(y0,u,v);
				int b1=YUV2B(y1,u,v);
				int g1=YUV2G(y1,u,v);
				int r1=YUV2R(y1,u,v);
				if(r0<0) r0=0;
				if(g0<0) g0=0;
				if(b0<0) b0=0;
				if(r0>255) r0=255;
				if(g0>255) g0=255;
				if(b0>255) b0=255;
				if(r1<0) r1=0;
				if(g1<0) g1=0;
				if(b1<0) b1=0;
				if(r1>255) r1=255;
				if(g1>255) g1=255;
				if(b1>255) b1=255;
				rgb[pd+(x*6)+0]=r0;
				rgb[pd+(x*6)+1]=g0;
				rgb[pd+(x*6)+2]=b0;
				rgb[pd+(x*6)+3]=r1;
				rgb[pd+(x*6)+4]=g1;
				rgb[pd+(x*6)+5]=b1;
			}
			p0+=s0;
			pd+=width*3;
		}
		p1+=s1;
	}
}

void* InitDecoderPlainH264(bool parseOnly) {
	NewDec::ISVCDecoderBase* pSvcDecoder=0;
	NewDec::CreateDecoder(&pSvcDecoder);
	NewDec::SDecodingParam sDecParam = {0};
	sDecParam.uiTargetDqLayer=0xff;
	sDecParam.eEcActiveIdc=NewDec::ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE;
	sDecParam.sVideoProperty.eVideoBsType=NewDec::VIDEO_BITSTREAM_DEFAULT;
	sDecParam.uiCpuLoad=100;
	pSvcDecoder->Initialize(&sDecParam);
	return pSvcDecoder;
}
void DestroyDecoderPlainH264(void* decoder) {
	NewDec::ISVCDecoderBase* pSvcDecoder=(NewDec::ISVCDecoderBase*)decoder;
	if(pSvcDecoder)
		delete pSvcDecoder;
	pSvcDecoder=0;
}

bool DecodeFramePlainH264Tensor(void* decoder, unsigned long long* ts, Tensor* decoded,const uint8_t* packetH264,int bytesizeH264,int* timerDecode,int* timerYuv2Rgb) {
	uint64_t t0=GetTimeEpochMicroseconds();
	NewDec::ISVCDecoderBase* pSvcDecoder=(NewDec::ISVCDecoderBase*)decoder;
	NewDec::SBufferInfo sDstBufInfo;
	memset(&sDstBufInfo, 0, sizeof(NewDec::SBufferInfo));
	*ts=*ts+1;
	sDstBufInfo.uiInBsTimeStamp = *ts;
	uint8_t* dst[3];
	NewDec::DECODING_STATE st=pSvcDecoder->DecodeFrame(packetH264,bytesizeH264,dst,&sDstBufInfo);
	if(st==NewDec::dsFramePending) {
		return false;
	}
	uint64_t t1=GetTimeEpochMicroseconds();
	bool success=false;
	if(st==NewDec::dsErrorFree) {
		if(sDstBufInfo.iBufferStatus==1) {

			int width=sDstBufInfo.UsrData.sSystemBuffer.iWidth;
			int height=sDstBufInfo.UsrData.sSystemBuffer.iHeight;
			int channels=3;
			int shape[] = {height, width, channels};
			int s0=sDstBufInfo.UsrData.sSystemBuffer.iStride[0];
			int s1=sDstBufInfo.UsrData.sSystemBuffer.iStride[1];
			switch(sDstBufInfo.UsrData.sSystemBuffer.iFormat) {
				case NewDec::videoFormatI420: {
					*decoded=CreateCPUTensor(3, shape, sizeof(uint8_t));
					uint8_t* rgb=(uint8_t*)decoded->m_data;
					ConvertYUV(rgb,dst[0],dst[1],dst[2],s0,s1,width,height,channels);
					success=true;
					break;
				}
				default:
					break;
			}
		}
	}
	if(timerDecode && timerYuv2Rgb) {
		uint64_t t2=GetTimeEpochMicroseconds();
		*timerDecode=(int)(t1-t0);
		*timerYuv2Rgb=(int)(t2-t1);
	}
	return success;
}

void PlainH264Impl::Begin(const Dict* config) {
	m_decoder=InitDecoderPlainH264(false);
}
bool PlainH264Impl::DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264) {
	return DecodeFramePlainH264Tensor(m_decoder,&m_ts,decoded,packetH264,bytesizeH264,&m_timers[0],&m_timers[1]);
}
void PlainH264Impl::End() {
	DestroyDecoderPlainH264(m_decoder);
	m_decoder=0;
}
void PlainH264Impl::Flush() {
	uprintf("PlainH264Impl::Flush\n");
	DestroyDecoderPlainH264(m_decoder);
	m_decoder=InitDecoderPlainH264(false);
}

int plainh264_static_link_use=0;
