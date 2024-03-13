#pragma once

#include <deque>
#include <map>
#include <stdint.h>
#include <functional>
#include "shared/net.h"
#include "shared/queue.h"
#include "shared/crc32.h"

#include "datasource.h"
#include "errorcorrection.h"

void ValidateTimeStamp(uint64_t time);

struct GraphDataset {
	std::string m_name;
	uint32_t m_color;
	struct Data {
		double m_x;
		double m_y;
	};
	std::vector<Data> m_data;
};

struct GraphDatasets {
	std::string m_name;
	std::string m_formatX;
	std::string m_formatY;
	std::vector<GraphDataset> m_datasets;
};

struct PacketHeaderBase {
	static uint8_t Magic(){return 0x78;}
	enum Type : uint8_t {
		NA=0,
		FROM_SERVER_FRAME_CHUNK=10,
		FROM_SERVER_FRAME_CHUNK_RESEND=11,
		FROM_CLIENT_PING=101,
		FROM_SERVER_PONG=102,
		FROM_SERVER_FRAME=103,
		FROM_CLIENT_REQUEST_FRAMES=104,
		FROM_CLIENT_REQUEST_MISSING_CHUNKS=106,

		GET_IDS=107,
		IDS=108,

		GET_STATUS=109,
		STATUS=110,

		GET_LOG=111,
		LOG=112,

		GET_PROPERTIES=113,
		SET_PROPERTIES=114,
		PROPERTIES=115,

		DEBUG_TIMING_REQUEST=116,
		DEBUG_TIMING_REPLY=117,

		CLOCK_MASTER_TIME_SEND=120,

		LAST_TYPE
	} m_type = NA;
	uint8_t m_magic=PacketHeaderBase::Magic();
	bool SanityCheck(int packetBytesize)const;
	bool IsDebugPacket()const{return m_type==SET_PROPERTIES || m_type==GET_PROPERTIES || m_type==GET_STATUS || m_type==STATUS || m_type==DEBUG_TIMING_REQUEST || m_type==DEBUG_TIMING_REPLY;}
};

struct PacketHeaderFrame : public PacketHeaderBase {
	PacketHeaderFrame(){m_type=FROM_SERVER_FRAME_CHUNK;m_magic=PacketHeaderBase::Magic();}
	uint8_t m_dataSourceType:DS_TYPE_BITS;
	uint8_t m_errorCorrectionType:EC_TYPE_BITS;
	uint8_t m_streamId:STREAM_ID_BITS;
	uint8_t m_last:1;
};

struct PacketHeader : public PacketHeaderBase {
	int PacketByteSize()const{return m_headerBytesize+m_dataBytesize;}
	uint16_t m_headerBytesize=sizeof(PacketHeader);
	uint32_t m_dataBytesize=0;
	uint64_t m_timeSend;
	bool SanityCheck(int packetBytesize)const;
	void* Data()const{return (char*)this+m_headerBytesize;}
};

struct PingFrameHeader : public PacketHeader {
	PingFrameHeader(){m_type=FROM_CLIENT_PING;m_headerBytesize=sizeof(*this);}
	uint32_t m_count=0;
};

struct PongFrameHeader : public PacketHeader {
	PongFrameHeader(){m_type=FROM_SERVER_PONG;m_headerBytesize=sizeof(*this);}
	uint64_t m_timeLoopback;
	uint32_t m_timeSpendServer=0;
	uint32_t m_count=0;
};
struct RequestFramesHeader : public PacketHeader {
	RequestFramesHeader(){m_type=FROM_CLIENT_REQUEST_FRAMES;m_headerBytesize=sizeof(*this);}
};

struct StatusLoopback {
	int16_t m_debugConnectionIndex=-1;
	int16_t m_debugServiceIndex=-1;
	int m_queryId=0;
};

struct GetDebugIdsHeader : public PacketHeader {
	GetDebugIdsHeader(){m_type=GET_IDS;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};
struct DebugIdsHeader : public PacketHeader {
	DebugIdsHeader(){m_type=IDS;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};

struct GetDebugStatusHeader : public PacketHeader {
	GetDebugStatusHeader(){m_type=GET_STATUS;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};
struct StatusHeader : public PacketHeader {
	StatusHeader(){m_type=STATUS;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};


struct GetDebugLogHeader : public PacketHeader {
	GetDebugLogHeader(){m_type=GET_LOG;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};
struct LogHeader : public PacketHeader {
	LogHeader(){m_type=LOG;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
	uint64_t m_timeLoopback=0;
	uint32_t m_timeSpendServer=0;
};

struct GetDebugPropertiesHeader : public PacketHeader {
	GetDebugPropertiesHeader(){m_type=GET_PROPERTIES;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};
struct SetDebugPropertiesHeader : public PacketHeader {
	SetDebugPropertiesHeader(){m_type=SET_PROPERTIES;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};
struct PropertiesHeader : public PacketHeader {
	PropertiesHeader(){m_type=PROPERTIES;m_headerBytesize=sizeof(*this);}
	StatusLoopback m_loopback;
};

struct SendFrameHeader : public PacketHeader {
	SendFrameHeader(){m_type=FROM_SERVER_FRAME;m_headerBytesize=sizeof(*this);}
	uint64_t m_captureTime=0;
	uint64_t m_sendTime=0;
	int m_width=0;
	int m_height=0;
	int m_index=0;
};

struct RequestMissingChunksHeader : public PacketHeader {
	RequestMissingChunksHeader(){m_type=FROM_CLIENT_REQUEST_MISSING_CHUNKS;m_headerBytesize=sizeof(*this);}
	uint8_t m_dataSourceType:DS_TYPE_BITS;
	uint8_t m_streamId:STREAM_ID_BITS;
};

//Debug timing
struct DebugTimingRequestHeader : public PacketHeader {
	DebugTimingRequestHeader(){m_type=DEBUG_TIMING_REQUEST;m_headerBytesize=sizeof(*this);}
	uint64_t m_captureTime=0;
};
struct DebugTimingReplyHeader : public PacketHeader {
	DebugTimingReplyHeader(){m_type=DEBUG_TIMING_REPLY;m_headerBytesize=sizeof(*this);}
	uint64_t m_timeLoopback=0;
	uint64_t m_captureTimeLoopback=0;
};

struct ClockMasterTimeSendHeader : public PacketHeader {
	ClockMasterTimeSendHeader(){m_type=CLOCK_MASTER_TIME_SEND;m_headerBytesize=sizeof(*this);}
	uint64_t m_masterClockMicroseconds;
	uint8_t m_dataSourceType:DS_TYPE_BITS;
	uint8_t m_streamId:STREAM_ID_BITS;
};
