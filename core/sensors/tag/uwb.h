#ifndef UWB_H_
#define UWB_H_

#include <stdint.h>
#include "shared/misc.h"

class Dict;

#define SETU48(b,i,v){b[i]=((v)>>32)&0xff;b[i+1]=((v)>>24)&0xff;b[i+2]=((v)>>16)&0xff;b[i+3]=((v)>>8)&0xff;b[i+4]=(v)&0xff;}
#define GETU48(b,i)((((uint64_t)b[i]<<32)|(uint64_t)b[i+1]<<24)|((uint64_t)b[i+2]<<16)|((uint64_t)b[i+3]<<8)|((uint64_t)b[i+4]))

#define SETU32(b,i,v){b[i]=((v)>>24)&0xff;b[i+1]=((v)>>16)&0xff;b[i+2]=((v)>>8)&0xff;b[i+3]=(v)&0xff;}
#define GETU32(b,i)((((uint32_t)b[i]<<24)|(uint32_t)b[i+1]<<16)|((uint32_t)b[i+2]<<8)|b[i+3])

#define SETU24(b,i,v){b[i]=((v)>>16)&0xff;b[i+1]=((v)>>8)&0xff;b[i+2]=(v)&0xff;}
#define GETU24(b,i)(((uint32_t)b[i]<<16)|((uint32_t)b[i+1]<<8)|(uint32_t)b[i+2])

#define SETU16(b,i,v){b[i]=((v)>>8)&0xff;b[i+1]=(v)&0xff;}
#define GETU16(b,i)(((uint32_t)b[i]<<8)|b[i+1])

template<class _T,const int _capacity> class Vector {
	public:
		Vector() {
			clear();
		}
		inline void push_back(const _T& val) {
			if(m_count>=_capacity)
				FATAL("Vector push_back overflow");
			m_array[m_count++]=val;
		}
		const _T& operator[](uint32_t ix)const{return m_array[ix];}
		_T& operator[](uint32_t ix){return m_array[ix]; }
		inline size_t size()const{return m_count;}
		inline uint32_t capacity()const{return _capacity;}
		inline uint32_t remain()const{return _capacity-m_count;}

		inline void remove(uint32_t ix) {
#if 1											// Keep order
			if(ix+1!=m_count)
				for(uint32_t i=ix;i!=m_count;i++) {
					m_array[i]=m_array[i+1];
				}
			--m_count;
#else
			m_array[ix]=m_array[--m_count];		// Copy last entry to removed entry
#endif
		}
		inline void clear() {
			m_count=0;
		}
	//protected:
		uint32_t m_count;
		_T m_array[_capacity];
};

#define MAX_NUMBER_TAGS 8
#define MAX_NUMBER_BASES 16
#define MAX_NUMBER_NEIGHBORS 8
#define MAX_STATUS_STRING_LENGTH 256

class StatusFlags {
	public:
		StatusFlags();
		void ToString(uint8_t* statusString)const;
		void SetFlag(uint32_t ix);
		uint32_t GetStatusMask()const;
		bool HasWarning()const;
	protected:
		uint64_t m_flagsTimes[32];
};

class UWBManager {
	public:
		enum eStatusFlag {
			PACKET_MAGIC=1,
			PACKET_LEN=2,
			RECEIVE_FAIL=3,
			TX_FAIL=4,
			RECEIVE_TIMEOUT=5,
			STATUS_FLAG_MAX=6
		};
		void SetFlag(eStatusFlag flag);
		const char* StatusFlagToName(eStatusFlag flag)const{				//return string max length=16
			switch(flag) {
				case PACKET_MAGIC: return "PACKET_MAGIC";
				case PACKET_LEN: return "PACKET_LEN";
				case RECEIVE_FAIL: return "RECEIVE_FAIL";
				case TX_FAIL: return "TX_FAIL";
				case RECEIVE_TIMEOUT: return "RECEIVE_TIMEOUT";
				default:
					break;
			}
			if((int)flag==0)
				return "FLAG0";
			if((int)flag==6)
				return "FLAG6";
			if((int)flag==7)
				return "FLAG7";

			return "NA";
		}
		StatusFlags m_status;
		bool HasWarning()const{return m_status.HasWarning();}
		uint32_t GetStatusMask()const{return m_status.GetStatusMask();}
};



class PingManager : public UWBManager {
	public:
		struct Measurements {
			struct Base {
				uint32_t m_id;
				float m_distance;
				uint8_t m_sequenceNr;
				uint64_t m_time;
				uint8_t m_frameCount=0;
				float m_nlos;
				struct Neighbor {
					uint32_t m_id;
					uint16_t m_dist;
				};
				Vector<Neighbor,MAX_NUMBER_NEIGHBORS> m_neighbors;
			};
			char m_status[MAX_STATUS_STRING_LENGTH];
			uint8_t m_frameCount;
			Vector<Base,MAX_NUMBER_BASES> m_bases;
		};
		PingManager();
		void Begin();
		uint32_t BeginPing();
		void SendPing(uint32_t index);
		void EndPing();
		uint32_t GetMeshCount()const{return (uint32_t)m_bases.size();}
		void GetMeasurements(Measurements* measurements);
		uint32_t m_dataSendBytesize=0;
		void Sanity()const;
	protected:
		void TimeoutBases();
		void SendTo(uint32_t id);
		void RegisterAndUpdateBase(uint32_t id,float distance,uint8_t sequenceNr,uint32_t otherBaseId,uint16_t otherBaseDist, float nlos);
		struct ActiveBase {
			uint32_t m_id=0;
			float m_distance=0;
			uint8_t m_sequenceNr=0;
			uint64_t m_lastAccessTime=0;
			uint8_t m_frameCount=0;
			uint8_t m_neighborSendIndex=0;
			float m_nlos=0;
			struct Neighbor {
				uint32_t m_id;
				uint16_t m_dist;
			};
			Vector<Neighbor,MAX_NUMBER_NEIGHBORS> m_neighbors;
			void RegisterNeighbor(uint32_t id,uint16_t dist);
		};
		uint32_t m_id;
		uint8_t m_frameCount;
		Vector<ActiveBase,MAX_NUMBER_BASES> m_bases;
};

class PongManager : public UWBManager {
	public:
		PongManager();
		void Begin();
		void Receive();
		uint32_t m_dataReceivedBytesize=0;
		uint32_t GetMeshCount()const{return (uint32_t)m_neighbors.size();}
		void PrintMesh();
	protected:
		void ReceiveData(const uint8_t* first64);
		uint32_t m_id;
		struct FrameTags {
			uint8_t m_neighborSendIndex;
			uint8_t m_seqNr;
			uint32_t m_id;
			uint8_t m_lastDataIndex;
		};
		Vector<FrameTags,MAX_NUMBER_TAGS> m_tags;
		uint8_t m_frameCount;
		void ResetFrameTags();
		void RegisterFrameTag(uint32_t id,uint8_t seqNr);
		bool CheckFrameTag(uint32_t id,uint8_t seqNr)const;
		bool ReadNextNeighborDistance(uint32_t* id,uint16_t* dist,uint32_t tagId);
		void RegisterNeighbor(uint32_t baseId);
		void UnregisterNeighbor(uint32_t baseId);
		struct Neighbor {
			uint32_t m_id;
			uint16_t m_dist;
			uint8_t m_continuousFailCount=0;
		};
		Vector<Neighbor,MAX_NUMBER_NEIGHBORS> m_neighbors;
		uint8_t m_neighborMeasureIndex=0;
		uint64_t m_lastNeighborPingTime=0;
		void MeasureDistanceToNeighbor();
};

#endif//UWB_H_
