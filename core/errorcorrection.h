#pragma once

#include <deque>
#include <string>
#include <stdint.h>

void TestErrorCorrection();

#define EC_TYPE_NA 0
#define EC_TYPE_FEC 1
#define EC_TYPE_ARQ 2

#define EC_TYPE_BITS 2

#define EC_TYPE_MAX (1<<EC_TYPE_BITS)

struct DebugData {
	uint64_t m_timeCapture;		//Time tag from server when data was captured
	int m_timers[2];
};

static inline const char* ErrorCorrectionTypeToName(uint8_t type){
	switch(type) {
		case EC_TYPE_FEC:return "fec";
		case EC_TYPE_ARQ:return "arq";
		default:
			break;
	}
	return "NA";
}

struct FecFrameHeader {
	int m_frameIndex;
	int m_frameBytesize;
	DebugData m_debugData;
	uint32_t m_CRC;
};

struct FECHeader {
	uint8_t m_k;
	uint8_t m_n;
	uint8_t m_fragmentIndex;
	uint8_t m_blokIndex;
	uint8_t m_blockCount;
	uint16_t m_blockSize;
	uint16_t m_fragmentSize;
	uint32_t m_bufferIndex;
	uint32_t m_packetIndex;
};

class FrameBase {
	public:
};

class ErrorCorrectionEncoder : public FrameBase {
	public:
		ErrorCorrectionEncoder(){}
		virtual ~ErrorCorrectionEncoder(){}
		virtual uint8_t Type()const=0;
		virtual void Timeout()=0;
		virtual void Encode(std::vector<std::vector<uint8_t>>* fragments,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint8_t streamId)=0;
		virtual void EncodeRetransmit(std::vector<std::vector<uint8_t>>* blocks,const uint8_t* data,int dataBytesize)=0;
};

typedef std::function<void(int index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint64_t timeFirst)> TBlocksDecodedCallbackFunc;
typedef std::function<void(const uint8_t* data,int dataBytesize)> TBlocksRetransmitRequestCallbackFunc;

class ErrorCorrectionDecoder : public FrameBase {
	public:
		ErrorCorrectionDecoder(){}
		virtual ~ErrorCorrectionDecoder(){}
		virtual int GetPacketDataIndex(const void* packet,int packetBytesize)const=0;
		virtual void Reset()=0;
		virtual void RequestRetransmit(uint8_t streamId,uint8_t streamType,const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksRetransmitRequestCallbackFunc retransmitCallback){}
		virtual void Decode(const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksDecodedCallbackFunc decodedCallback)=0;
		virtual void GetGraphs(Dict* graphs,uint64_t timeMilliseconds,uint8_t streamId)=0;
		struct Statistics {
			int m_framesLost=0;
			int m_framesDecoded=0;

			int m_packetsLost=0;
			int m_packetsReceived=0;
			int m_bytesReceived=0;
			int m_lastPacketsLost=0;
			int m_lastPacketsReceived=0;
			int m_lastBytesReceived=0;
			int m_lastFramesLost=0;
			int m_lastFramesDecoded=0;
			uint64_t m_lastTime=0;
		};
		virtual std::string GetStatistics(Statistics* statistics)const{return "";}
};

ErrorCorrectionEncoder* CreateErrorCorrectionEncoder(const Dict* settings);
void DestroyErrorCorrectionEncoder(ErrorCorrectionEncoder*);

ErrorCorrectionDecoder* CreateErrorCorrectionDecoder(const Dict* settings);
void DestroyErrorCorrectionDecoder(ErrorCorrectionDecoder*);
ErrorCorrectionDecoder* CreateErrorCorrectionDecoder(uint8_t errorCorrectionType);

