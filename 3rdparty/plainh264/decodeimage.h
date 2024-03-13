#pragma once

#include "shared/types.h"

namespace NewDec {

// Enumerate the type of video bitstream which is provided to decoder
typedef enum{
	VIDEO_BITSTREAM_AVC=0,
	VIDEO_BITSTREAM_SVC=1,
	VIDEO_BITSTREAM_DEFAULT=VIDEO_BITSTREAM_SVC
} VIDEO_BITSTREAM_TYPE;


// Define a new struct to show the property of video bitstream.
typedef struct{
	unsigned int          size;				// size of the struct
	VIDEO_BITSTREAM_TYPE  eVideoBsType;		// video stream type (AVC/SVC)
} SVideoProperty;

// Enumerate the type of error concealment methods
typedef enum{
	ERROR_CON_DISABLE=0,
	ERROR_CON_FRAME_COPY,
	ERROR_CON_SLICE_COPY,
	ERROR_CON_FRAME_COPY_CROSS_IDR,
	ERROR_CON_SLICE_COPY_CROSS_IDR,
	ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE,
	ERROR_CON_SLICE_MV_COPY_CROSS_IDR,
	ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE
} ERROR_CON_IDC;

// SVC Decoding Parameters, reserved here and potential applicable in the future
typedef struct TagSVCDecodingParam{
	char* pFileNameRestructed;				// file name of reconstructed frame used for PSNR calculation based debug
	unsigned int uiCpuLoad;  				// CPU load
	unsigned char uiTargetDqLayer;			// setting target dq layer id
	ERROR_CON_IDC eEcActiveIdc;				// whether active error concealment feature in decoder
	bool bParseOnly;          				// decoder for parse only, no reconstruction. When it is true, SPS/PPS size should not exceed SPS_PPS_BS_SIZE (128). Otherwise, it will return error info
	SVideoProperty sVideoProperty;			// video stream property
} SDecodingParam,* PDecodingParam;

// Enumerate the type of video format
typedef enum{
	videoFormatRGB=1,					  	// rgb color formats
	videoFormatRGBA=2,
	videoFormatRGB555=3,
	videoFormatRGB565=4,
	videoFormatBGR=5,
	videoFormatBGRA=6,
	videoFormatABGR=7,
	videoFormatARGB=8,
	videoFormatYUY2=20,						// yuv color formats
	videoFormatYVYU=21,
	videoFormatUYVY=22,
	videoFormatI420=23, 					// the same as IYUV
	videoFormatYV12=24,
	videoFormatInternal=25,					// only used in SVC decoder testbed
	videoFormatNV12=26, 					// new format for output by DXVA decoding
	videoFormatVFlip=0x80000000
} EVideoFormatType;

// Decoding status
typedef enum{
// Errors derived from bitstream parsing
	dsErrorFree=0x00,						// bit stream error-free
	dsFramePending=0x01,					// need more throughput to generate a frame output,
	dsRefLost=0x02,							// layer lost at reference frame with temporal id 0
	dsBitstreamError=0x04,					// error bitstreams(maybe broken internal frame) the decoder cared
	dsDepLayerLost=0x08,					// dependented layer is ever lost
	dsNoParamSets=0x10,						// no parameter set NALs involved
	dsDataErrorConcealed=0x20,				// current data error concealed specified
	dsRefListNullPtrs=0x40,					// ref picure list contains null ptrs within uiRefCount range
// Errors derived from logic level
	dsInvalidArgument=0x1000,				// invalid argument specified
	dsInitialOptExpected=0x2000,			// initializing operation is expected
	dsOutOfMemory=0x4000,					// out of memory due to new request
// ANY OTHERS?
	dsDstBufNeedExpan=0x8000				// actual picture size exceeds size of dst pBuffer feed in decoder, so need expand its size
} DECODING_STATE;

// Option types introduced in decoder application
typedef enum{
	DECODER_OPTION_END_OF_STREAM=1,			// end of stream flag
	DECODER_OPTION_VCL_NAL,    				// feedback whether or not have VCL NAL in current AU for application layer
	DECODER_OPTION_TEMPORAL_ID,				// feedback temporal id for application layer
	DECODER_OPTION_FRAME_NUM,				// feedback current decoded frame number
	DECODER_OPTION_IDR_PIC_ID, 				// feedback current frame belong to which IDR period
	DECODER_OPTION_LTR_MARKING_FLAG,		// feedback wether current frame mark a LTR
	DECODER_OPTION_LTR_MARKED_FRAME_NUM,	// feedback frame num marked by current Frame
	DECODER_OPTION_ERROR_CON_IDC,			// indicate decoder error concealment method
	DECODER_OPTION_TRACE_LEVEL,
	DECODER_OPTION_TRACE_CALLBACK,			// a void (*)(void* context, int level, const char* message) function which receives log messages
	DECODER_OPTION_TRACE_CALLBACK_CONTEXT,	// context info of trace callbac

	DECODER_OPTION_GET_STATISTICS,			// feedback decoder statistics
	DECODER_OPTION_GET_SAR_INFO,			// feedback decoder Sample Aspect Ratio info in Vui
	DECODER_OPTION_PROFILE,    				// get current AU profile info, only is used in GetOption
	DECODER_OPTION_LEVEL,      				// get current AU level info,only is used in GetOption
	DECODER_OPTION_STATISTICS_LOG_INTERVAL,	// set log output interval
	DECODER_OPTION_IS_REF_PIC,  			// feedback current frame is ref pic or not
	DECODER_OPTION_NUM_OF_FRAMES_REMAINING_IN_BUFFER,// number of frames remaining in decoder buffer when pictures are required to re-ordered into display-order.
	DECODER_OPTION_NUM_OF_THREADS,			// number of decoding threads. The maximum thread count is equal or less than lesser of (cpu core counts and 16).
} DECODER_OPTION;

//  Structure for decoder memery
typedef struct TagSysMemBuffer{
	int iWidth;								// width of decoded pic for display
	int iHeight;							// height of decoded pic for display
	int iFormat;							// type is "EVideoFormatType"
	int iStride[2];							// stride of 2 component
} SSysMEMBuffer;

//  Buffer info
typedef struct TagBufferInfo{
	int iBufferStatus;						// 0: one frame data is not ready; 1: one frame data is ready
	unsigned long long uiInBsTimeStamp;		// input BS timestamp
	unsigned long long uiOutYuvTimeStamp;	// output YUV timestamp, when bufferstatus is 1
	union{
		SSysMEMBuffer sSystemBuffer;		// memory info for one picture
	} UsrData;          					// output buffer info
	unsigned char* pDst[3];					// point to picture YUV data
} SBufferInfo;

class ISVCDecoderBase{
	public:
		virtual ~ISVCDecoderBase(){}
		virtual DECODING_STATE DecodeFrame(const unsigned char* pSrc,const int iSrcLen,unsigned char** ppDst,SBufferInfo* pDstInfo)=0;
		virtual long SetOption(DECODER_OPTION eOptID,void* pOption)=0;
		virtual long Initialize(const SDecodingParam* pParam)=0;
};
long CreateDecoder(ISVCDecoderBase** ppDecoder);
};