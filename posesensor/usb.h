#pragma once

enum eStatusFlag {
  PACKET_MAGIC=1,
  PACKET_LEN=2,
  RECEIVE_FAIL=3,
  RECEIVE_TIMEOUT=4,
  TX_FAIL=5,
  IMU_RING_FULL=6,
  RECEIVE_FAIL_DATA=7
};

enum ePacketType {
	PT_IMU1=1,
	PT_SYS1=2,
	PT_UWB1=3
};

inline const char* PacketTypeToName(ePacketType type) {
	switch(type) {
		case PT_IMU1: return "IMU1";
		case PT_SYS1: return "SYS1";
		case PT_UWB1: return "UWB1";
	}
	return "NA";
}

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif
uint8_t GetSeqIndex();

PACK(struct Packet {
	uint8_t m_type;
	uint8_t m_byteSize;
	uint8_t m_seqIndex;
	uint32_t ToString(char* buf,uint32_t bufBytesize,uint64_t time)const;
});

PACK(struct IMU1Packet : public Packet {
	IMU1Packet(){m_type=PT_IMU1;m_byteSize=sizeof(*this);m_seqIndex=GetSeqIndex();}
	uint32_t ToString(char* buf,uint32_t bufBytesize,uint64_t time)const;
	uint64_t m_time;
	float m_acc[3];
	float m_rads[3];
	float m_mags[3];
	float m_temp[2];
	uint32_t m_dt;
});

PACK(struct SYS1Packet : public Packet {
	SYS1Packet(){m_type=PT_SYS1;m_byteSize=sizeof(*this);m_seqIndex=GetSeqIndex();}
	uint32_t ToString(char* buf,uint32_t bufBytesize,uint64_t time)const;
	uint64_t m_time;
	uint32_t m_status;
	uint32_t m_dataSendBytesize;
	uint32_t m_dataReceivedBytesize;
	uint32_t m_meshCount;
});

PACK(struct UWB1Packet : public Packet {
	UWB1Packet(){m_type=PT_UWB1;m_byteSize=sizeof(*this);m_seqIndex=GetSeqIndex();}
	uint32_t ToString(char* buf,uint32_t bufBytesize,uint64_t time)const;
	uint64_t m_time;
	uint32_t m_id;
	float m_distance;
	uint32_t m_irqCount;
	uint8_t m_neighborIndex;
	uint32_t m_neighborId;
	uint16_t m_neighborDist;
});

class PacketStream {
	public:
		PacketStream();
		const Packet* Data()const{return (const Packet*)m_data;}
		uint32_t Size()const{return m_byteSize;}
		void Push(const Packet& packet);
		void Flush(uint64_t time);				//Send and flush
		void Flush();							//Flush
	protected:
		uint32_t m_byteSize;
		uint8_t m_data[4096];
};
