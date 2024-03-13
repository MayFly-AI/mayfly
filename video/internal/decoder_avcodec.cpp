#if AVDECODE
#include <string>
#include "video/decoder.h"
#include "memory/tensor.h"
#include "shared/dict.h"
#include "shared/misc.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

class AvDecodeImpl : public H264Decoder{
public:
	virtual ~AvDecodeImpl(){}
	virtual void Begin(const Dict* config);
	virtual bool DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264);
	virtual void End();
	virtual void Flush();
	virtual int Channels()const{return PIXEL_FORMAT==AV_PIX_FMT_RGBA?4:3;}

	static H264Decoder* Create() { return new AvDecodeImpl(); }
private:
	static void UpdateCachedFrame(AVFrame** frame,int width,int height,int format);
	AVCodecContext* m_ctx{0};
	AVPacket* m_pkt{0};
	AVFrame* m_frame{0};
	AVFrame* m_frameOut{0};
	AVCodecParserContext* m_parser{0};
	SwsContext* m_sws_ctx{0};
	uint64_t m_count{0};
	std::vector<uint8_t> m_imageBuffer;
	bool m_naluHack{false};
	int m_width{0};
	int m_height{0};
	//const AVPixelFormat PIXEL_FORMAT = AV_PIX_FMT_RGB24;
	const AVPixelFormat PIXEL_FORMAT = AV_PIX_FMT_RGBA ;
	static const bool s_registered;
};
const bool AvDecodeImpl::s_registered=RegisterH264Decoder("avdecode",AvDecodeImpl::Create);

void AvDecodeImpl::UpdateCachedFrame(AVFrame** frame,int width,int height,int format) {
	ASSERT(frame,"null pointer passed");
	AVFrame* frm=*frame;
	if(frm && frm->width==width && frm->height==height && frm->format==format) {
		return;
	}
	if(frm) {
		av_frame_free(frame);
	}
	frm=av_frame_alloc();
	frm->width=width;
	frm->height=height;
	frm->format=format;
	int ret=av_frame_get_buffer(frm,0);
	if(!ret) {
		*frame=frm;
		return;
	}
	*frame=0;
	FATAL("Failed to allocate AVFrame");
}

bool AvDecodeImpl::DecodeFrame(Tensor* decoded,const uint8_t* packetH264,int bytesizeH264){
	m_timers[0]=0;
	m_timers[1]=0;

	int size = 0;
	if(m_naluHack) {
		const char* nalu = "\00\00\00\01";
		char* pc = (char*)packetH264;
		if(m_count==0){
			m_imageBuffer.resize(bytesizeH264+4+AV_INPUT_BUFFER_PADDING_SIZE);
			memcpy(m_imageBuffer.data(),pc,bytesizeH264);
			memcpy(m_imageBuffer.data()+bytesizeH264,nalu,4);
			size = bytesizeH264+4;
		} else {
			if(memcmp(pc,nalu,4)) {
				FATAL("NALU hack expected 0001 at package start");
			}
			m_imageBuffer.resize(bytesizeH264+AV_INPUT_BUFFER_PADDING_SIZE);
			memcpy(m_imageBuffer.data(),pc+4,bytesizeH264-4);
			memcpy(m_imageBuffer.data()+bytesizeH264-4,nalu,4);
			size = bytesizeH264;
		}
	} else {
		m_imageBuffer.resize(bytesizeH264+AV_INPUT_BUFFER_PADDING_SIZE);
		memcpy(m_imageBuffer.data(),packetH264,bytesizeH264);
		size=bytesizeH264;
	}
	const uint8_t* data = m_imageBuffer.data();

	uint64_t t0=GetTimeEpochMicroseconds();
	bool frame_ok=false;
	while(size > 0){
		int eat=av_parser_parse2(m_parser,m_ctx,&m_pkt->data,&m_pkt->size,data,size,AV_NOPTS_VALUE,AV_NOPTS_VALUE,0);
		if(eat<0){
			FATAL("Error while parsing");
		}
		data+=eat;
		size-=eat;
		if(m_pkt->size){
			int send_recv=avcodec_send_packet(m_ctx,m_pkt);
			if(send_recv<0){
				m_timers[0]=GetTimeEpochMicroseconds()-t0;
				return false;
			}

			if(send_recv>=0){
				int send_recv=avcodec_receive_frame(m_ctx,m_frame);
				if(send_recv==AVERROR(EAGAIN)||send_recv==AVERROR_EOF){
					break;
				}
				else if(send_recv<0){
					FATAL("Error during decoding");
				}
				m_sws_ctx=sws_getCachedContext(m_sws_ctx,m_ctx->width,m_ctx->height,m_ctx->pix_fmt,m_ctx->width,m_ctx->height,PIXEL_FORMAT,SWS_FAST_BILINEAR,0,0,0);
				UpdateCachedFrame(&m_frameOut,m_ctx->width,m_ctx->height,PIXEL_FORMAT);

				int sts=sws_scale(m_sws_ctx,m_frame->data,m_frame->linesize,0,m_frame->height,m_frameOut->data,m_frameOut->linesize);
				if(sts!=m_frame->height){
					FATAL("Error unexpected height");
				}

				int shape[] = {m_frame->height,m_frame->width,Channels()};
				*decoded=CreateCPUTensor(3,shape,sizeof(uint8_t));
				uint8_t* ptr=(uint8_t*)decoded->m_data;
				ASSERT(m_frameOut->linesize[0] == decoded->m_elementSize*decoded->m_stride[0],"unexpected stride\n");
				memcpy(ptr,m_frameOut->data[0],m_frameOut->linesize[0]*m_frameOut->height);
				frame_ok=true;
			}
		}
	}
	m_timers[0]=GetTimeEpochMicroseconds()-t0;
	m_count++;
	return frame_ok;
}
void AvDecodeImpl::Begin(const Dict* config){
	if(config) {
		config->Get("naluHack",&m_naluHack,false);
		if(m_naluHack) {
			uprintf("using NALU hack\n");
		}
	}

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58,9,100)
	avcodec_register_all();
#endif
	const AVCodec* codec=avcodec_find_decoder(AV_CODEC_ID_H264);
	m_ctx=avcodec_alloc_context3(codec);
	if(avcodec_open2(m_ctx,codec,NULL) < 0){
		FATAL("Error opening codec");
	}
	m_pkt=av_packet_alloc();
	m_frame=av_frame_alloc();
	m_parser=av_parser_init(codec->id);
	m_count=0;
	uprintf("AV decode init ok!\n");
}
void AvDecodeImpl::End(){
	av_frame_free(&m_frame);
	av_packet_free(&m_pkt);
	av_parser_close(m_parser);
	avcodec_free_context(&m_ctx);
	sws_freeContext(m_sws_ctx);
	av_frame_free(&m_frameOut);
}
void AvDecodeImpl::Flush() {
	End();
	Begin(nullptr);
}
#endif//AVDECODE
int avdecode_static_link_use=0;
