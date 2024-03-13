#ifdef CUDA
#include "video/decoder.h"
#include <string>
#include "memory/cuda_alloc.h"
#include "memory/tensor.h"
#include "shared/dict.h"
#include "shared/misc.h"
#include <cuda.h>
#include "NvDecoder/NvDecoder.h"
#include "3rdparty/nv_video_codec/Utils/ColorSpace.h"

class NvDecodeImpl : public H264Decoder {
public:
	virtual ~NvDecodeImpl(){}
	virtual void Begin(const Dict* config);
	virtual bool DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264);
	virtual void End();
	virtual void Flush();
	virtual int Channels()const{return 4;}

	static H264Decoder* Create() { return new NvDecodeImpl(); }
protected:
	void GetImage(void* dpSrc,uint8_t* pDst,int nWidth,int nHeight);
	CUcontext m_cuContext=NULL;
	NvDecoder* m_dec;
	int m_nFrame=0;
	int m_nWidth=0;
	int m_nHeight=0;
	Tensor m_tensor;
	bool m_cudaDevice=false;
	static const bool s_registered;
};

const bool NvDecodeImpl::s_registered=RegisterH264Decoder("nvdecode", NvDecodeImpl::Create);

void NvDecodeImpl::GetImage(void* dpSrc,uint8_t* pDst,int nWidth,int nHeight) {
	CUDA_MEMCPY2D m={0};
	m.WidthInBytes=nWidth;
	m.Height=nHeight;
	m.srcMemoryType=CU_MEMORYTYPE_DEVICE;
	m.srcDevice=(CUdeviceptr)dpSrc;
	m.srcPitch=m.WidthInBytes;
	m.dstMemoryType=CU_MEMORYTYPE_HOST;
	m.dstDevice=(CUdeviceptr)(m.dstHost=pDst);
	m.dstPitch=m.WidthInBytes;
	cuMemcpy2D(&m);
}
void NvDecodeImpl::Begin(const Dict* config) {
	std::string device;
	config->Get("device",&device);
	m_cudaDevice=device=="cuda";

	int iGpu=0;
	ck(cuInit(0));
	int nGpu=0;
	ck(cuDeviceGetCount(&nGpu));
	if(iGpu < 0 || iGpu >=nGpu) {
		FATAL("GPU ordinal out of range. Should be within [0,%d]\n",nGpu-1);
	}
	CUdevice cuDevice = 0;
	ck(cuDeviceGet(&cuDevice,iGpu));
	char szDeviceName[80];
	ck(cuDeviceGetName(szDeviceName,sizeof(szDeviceName),cuDevice));
	uprintf("GPU in use: %s\n",szDeviceName);
	ck(cuCtxCreate(&m_cuContext,0,cuDevice));
	m_dec=new NvDecoder(m_cuContext,true,cudaVideoCodec_H264,true,false,0,0,false,0,0,1000,true);
}

void NvDecodeImpl::Flush() {
	if(!m_nFrame)
		return;		//No need decoder not started
	delete m_dec;
	m_dec=new NvDecoder(m_cuContext,true,cudaVideoCodec_H264,true,false,0,0,false,0,0,1000,true);
}

bool NvDecodeImpl::DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264) {
	uint64_t t0=GetTimeEpochMicroseconds();
	int nFrameReturned=m_dec->Decode(packetH264,bytesizeH264);
	uint64_t t1=GetTimeEpochMicroseconds();
	if(!nFrameReturned) {
		uint64_t t2=GetTimeEpochMicroseconds();
		m_timers[0]=(int)(t1-t0);
		m_timers[1]=(int)(t2-t1);
		return false;
	}
	if(nFrameReturned!=1)
		uprintf("NvDecodeImpl::DecodeFrame decode more than one frame. Should not be possible\n");
	cuCtxSetCurrent(m_cuContext);

	if(m_nHeight != m_dec->GetHeight() || m_nWidth != m_dec->GetWidth()) {
		m_tensor.Free();
	}

	if(!m_tensor.m_data) {
		int sizes[3] = {m_dec->GetHeight(), m_dec->GetWidth(), 4};
		m_nHeight = m_dec->GetHeight();
		m_nWidth = m_dec->GetWidth();
		m_tensor = CreateCUDATensor(3, sizes, sizeof(uint8_t));
	}
	//for(int i=0;i<nFrameReturned;i++) {
		int iMatrix=m_dec->GetVideoFormatInfo().video_signal_description.matrix_coefficients;
		uint8_t* pFrame=m_dec->GetFrame();
		if(m_dec->GetOutputFormat()==cudaVideoSurfaceFormat_YUV444)
			FATAL("Support this");
		Nv12ToColor32Alpha<RGBA32>(pFrame,m_dec->GetWidth(),(uint8_t*)m_tensor.m_data,4*m_dec->GetWidth(),m_dec->GetWidth(),m_dec->GetHeight(),iMatrix,0xff);
		if(!m_cudaDevice) {
			int sizes[3] = {m_nHeight, m_nWidth, 4};
			Tensor tensor = CreateCPUTensor(3, sizes, sizeof(uint8_t));
			uint8_t* pixelsFrame=(uint8_t*)tensor.m_data;
			GetImage(m_tensor.m_data,pixelsFrame,4*m_dec->GetWidth(),m_dec->GetHeight());
			*decoded=std::move(tensor);
		}else{
			*decoded=std::move(m_tensor);
		}
	//}
	uint64_t t2=GetTimeEpochMicroseconds();
	m_timers[0]=(int)(t1-t0);
	m_timers[1]=(int)(t2-t1);
	m_nFrame+=nFrameReturned;
	return true;
}

void NvDecodeImpl::End() {
	delete m_dec;
	cuCtxDestroy(m_cuContext);
}

#endif//CUDA

int nvdecode_static_link_use=0;
