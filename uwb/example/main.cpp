#include <thread>
#include <atomic>
#include <math.h>
#include <iostream>

#include "shared/queue.h"
#include "shared/math.h"
#include "shared/std_ext.h"

#include "platform/gpio.h"
#include "platform/spi.h"

#include "uwb/dw_regs.h"
#include "uwb/platform_io.h"
#include "uwb/deca_spi.h"
#include "uwb/deca_dw_device_api.h"
#include "uwb/dw_device.h"

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
		uint32_t m_count;
		_T m_array[_capacity];
};

#define MAX_NUMBER_TAGS 8
#define MAX_NUMBER_BASES 16
#define MAX_NUMBER_NEIGHBORS 8
#define MAX_STATUS_STRING_LENGTH 256

// connection pins (BROADCOM PIN NUMBERING)
const uint8_t PIN_RST = 23; // reset pin
const uint8_t PIN_IRQ = 24; // irq pin
const uint8_t PIN_CE0 = 8; // spi select pin

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
		PingManager();
		void Begin();
		uint32_t BeginPing();
		void SendPing(uint32_t index);
		void EndPing();
		uint32_t GetMeshCount()const{return (uint32_t)m_bases.size();}
		void PrintMeasurements();
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

int main(int argc,char *argv[]) {

	std::atomic<bool> ready=false;

	RPI_GPIO::setup();
	rpi_spi_init();
	RPI_GPIO::setup_gpio(PIN_CE0,RPI_GPIO::PIN_MODE::OUTPUT,0);
	RPI_GPIO::output_gpio(PIN_CE0,1);

	dw_irq_init(PIN_IRQ);
	deca_usleep(2000);
	reset_DWIC(PIN_RST);

	ready=true;
	PingManager pm;
	pm.Begin();
	while(true) {
		uint64_t t64s0=GetTimeMicroseconds();
		uint32_t count=pm.BeginPing();
		for(uint32_t i=0;i!=count;i++) {
			pm.SendPing(i);
		}
		pm.EndPing();
		if(pm.HasWarning()) {
			uprintf("PingManager status flags $%08x\n",pm.GetStatusMask());
		}
		pm.PrintMeasurements();
		while(true){
			uint64_t t64=GetTimeMicroseconds();
			if(t64-t64s0>500000L)						//Ping frequency
				break;
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}
	}
}

uint32_t GetUniqueId24(){
	return 0x123456;
}

StatusFlags::StatusFlags() {
	memset(m_flagsTimes,0,sizeof(m_flagsTimes));
}
bool StatusFlags::HasWarning()const{
	uint64_t t=GetTimeMicroseconds();
	for(int i=0;i!=countof(m_flagsTimes);i++) {
		if(m_flagsTimes[i]!=0 && t-m_flagsTimes[i]<1000000)
			return true;
	}
	return false;
}
void StatusFlags::SetFlag(uint32_t ix) {
	if(ix>countof(m_flagsTimes))
		FATAL("StatusFlags::SetFlag %d out of range",ix);
	m_flagsTimes[ix]=GetTimeMicroseconds();
}
uint32_t StatusFlags::GetStatusMask()const {
	uint32_t status=0;
	uint64_t t=GetTimeMicroseconds();
	for(int i=0;i!=countof(m_flagsTimes);i++) {
		if(m_flagsTimes[i]!=0 && t-m_flagsTimes[i]<1000000)
			status|=1<<i;
	}
	return status;
}

void UWBManager::SetFlag(eStatusFlag flag) {
	//uprintf("PongManager::SetFlag %s\n",StatusFlagToName(flag));
	m_status.SetFlag((uint32_t)flag);
}

void NoPrint(const char* format,...){}
#define noprint(...)NoPrint(__VA_ARGS__)				// To avoid warnings

#define UWB_NOTIFY noprint
#define UWB_WARNING uprintf
#define UWB_ERROR FATAL

// Default communication configuration. We use default non-STS DW mode.
static dwt_config_t config={
	5,					// Channel number.
	DWT_PLEN_128,		// Preamble length. Used in TX only.
	DWT_PAC8,			// Preamble acquisition chunk size. Used in RX only.
	9,					// TX preamble code. Used in TX only.
	9,					// RX preamble code. Used in RX only.
	(dwt_sfd_type_e)1,	// 0 to use standard 8 symbol SFD,1 to use non-standard 8 symbol,2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type
	DWT_BR_6M8,			// Data rate.
	DWT_PHRMODE_STD,	// PHY header mode.
	DWT_PHRRATE_STD,	// PHY header rate.
	(129+8-8),			// SFD timeout (preamble length+1+SFD length - PAC size). Used in RX only.
	DWT_STS_MODE_OFF,	// STS disabled
	DWT_STS_LEN_64,		// STS length see allowed values in Enum dwt_sts_lengths_e
	DWT_PDOA_M0			// PDOA mode off
};

// Default antenna delay values for 64 MHz PRF
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

// Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current temperature. These values can be calibrated prior to taking reference measurements.
static dwt_txconfig_t txconfig_options={
	0x34,			// PG delay
	0xfdfdfdfd,		// TX power
	0x0				// PG count
};

#define DELAY_OFFSET ((550*UUS_TO_DWT_TIME)+TX_ANT_DLY)

#define MESH_ID 0x42

#define VERSION 0x89
#define MAGIC 0x55

#define OFFSET_MESH_ID 0
#define OFFSET_VERSION 1
#define OFFSET_SEQ_IX 2
#define OFFSET_MAGIC 3
#define OFFSET_SOURCE_ID 4
#define OFFSET_TARGET_ID 7
#define OFFSET_BASE_DELAY_IDX 10
#define OFFSET_INTEGRATOR_IDX 14

bool DefaultValidate(const uint8_t* buffer) {
	if(buffer[3]!=MAGIC)
		return false;
	if(buffer[1]!=VERSION)
		return false;
	if(buffer[0]!=MESH_ID)
		return false;
	return true;
}

struct TagToBaseDataFirst {
	uint8_t m_data[64];
	inline bool Validate()const {
		return DefaultValidate(m_data);
	}
	inline uint8_t GetSequenceIndex()const {
		return m_data[2];
	}
	inline void SetSequenceIndex(uint8_t ix) {
		m_data[2]=ix;
	}
	inline uint32_t GetSourceId() {
		return GETU24(m_data,4);
	}
	inline void SetSourceId(uint32_t id) {
		SETU24(m_data,4,id);
	}
};

struct TagToBaseDistanceRequest {
	TagToBaseDistanceRequest() {
		memset(m_data,0,sizeof(m_data));
		m_data[OFFSET_MESH_ID]=MESH_ID;
		m_data[OFFSET_VERSION]=VERSION;
		m_data[OFFSET_MAGIC]=MAGIC;
	}
	uint8_t m_data[12];
	inline bool Validate()const {
		return DefaultValidate(m_data);
	}
	inline uint8_t GetSequenceIndex()const {
		return m_data[2];
	}
	inline void SetSequenceIndex(uint8_t ix) {
		m_data[2]=ix;
	}
	inline uint32_t GetTargetId() {
		return GETU24(m_data,7);
	}
	inline void SetTargetId(uint32_t id) {
		SETU24(m_data,7,id);
	}
	inline uint32_t GetSourceId() {
		return GETU24(m_data,4);
	}
	inline void SetSourceId(uint32_t id) {
		SETU24(m_data,4,id);
	}
};
struct BaseToTagDistanceResponse {
	BaseToTagDistanceResponse() {
		memset(m_data,0,sizeof(m_data));
		m_data[OFFSET_MESH_ID]=MESH_ID;
		m_data[OFFSET_VERSION]=VERSION;
		m_data[OFFSET_MAGIC]=MAGIC;
	}
	uint8_t m_data[28];
	inline bool Validate()const {
		return DefaultValidate(m_data);
	}
	inline void SetNeighborDistance(uint32_t id,uint16_t dist) {
		SETU24(m_data,18,id);
		SETU16(m_data,21,dist);
	}
	inline uint32_t GetNeighborDistance(uint16_t* dist) {
		*dist=GETU16(m_data,21);
		return GETU24(m_data,18);
	}
	inline uint8_t GetSequenceIndex()const {
		return m_data[2];
	}
	inline void SetSequenceIndex(uint8_t ix) {
		m_data[2]=ix;
	}
	inline uint32_t GetSourceId() {
		return GETU24(m_data,4);
	}
	inline void SetSourceId(uint32_t id) {
		SETU24(m_data,4,id);
	}
	inline void SetDelay(int32_t delay) {
		SETU32(m_data,OFFSET_BASE_DELAY_IDX,delay);
	}
	inline void SetOffset(int16_t offset) {
		SETU16(m_data,OFFSET_INTEGRATOR_IDX,offset);
	}
	inline int32_t GetDelay() {
		return GETU32(m_data,OFFSET_BASE_DELAY_IDX);
	}
	inline int16_t GetOffset() {
		return (int16_t)GETU16(m_data,OFFSET_INTEGRATOR_IDX);
	}

};

struct BaseToBaseDistanceRequest {
	BaseToBaseDistanceRequest() {
		memset(m_data,0,sizeof(m_data));
		m_data[OFFSET_MESH_ID]=MESH_ID;
		m_data[OFFSET_VERSION]=VERSION;
		m_data[OFFSET_MAGIC]=MAGIC;
	}
	uint8_t m_data[16];
	inline bool Validate()const {
		return DefaultValidate(m_data);
	}
	inline void SetSequenceIndex(uint8_t ix) {
		m_data[2]=ix;
	}
	inline uint32_t GetTargetId() {
		return GETU24(m_data,7);
	}
	inline void SetTargetId(uint32_t id) {
		SETU24(m_data,7,id);
	}
	inline uint32_t GetSourceId() {
		return GETU24(m_data,4);
	}
	inline void SetSourceId(uint32_t id) {
		SETU24(m_data,4,id);
	}
};
struct BaseToBaseDistanceResponse {
	BaseToBaseDistanceResponse() {
		memset(m_data,0,sizeof(m_data));
		m_data[OFFSET_MESH_ID]=MESH_ID;
		m_data[OFFSET_VERSION]=VERSION;
		m_data[OFFSET_MAGIC]=MAGIC;
	}
	uint8_t m_data[20];
	inline bool Validate()const {
		return DefaultValidate(m_data);
	}
	inline void SetSequenceIndex(uint8_t ix) {
		m_data[2]=ix;
	}
	inline void SetSourceId(uint32_t id) {
		SETU24(m_data,4,id);
	}
	inline void SetDelay(int32_t delay) {
		SETU32(m_data,OFFSET_BASE_DELAY_IDX,delay);
	}
	inline void SetOffset(int16_t offset) {
		SETU16(m_data,OFFSET_INTEGRATOR_IDX,offset);
	}
	inline int32_t GetDelay() {
		return GETU32(m_data,OFFSET_BASE_DELAY_IDX);
	}
	inline int16_t GetOffset() {
		return (int16_t)GETU16(m_data,OFFSET_INTEGRATOR_IDX);
	}
};

PingManager::PingManager() {
	m_frameCount=0;
	m_id=GetUniqueId24();

	if(sizeof(TagToBaseDistanceRequest)==sizeof(BaseToBaseDistanceRequest))
		FATAL("MUST BE DIFFERENT");
	if(sizeof(TagToBaseDistanceRequest)==sizeof(BaseToTagDistanceResponse))
		FATAL("MUST BE DIFFERENT");
	if(sizeof(TagToBaseDistanceRequest)==sizeof(BaseToBaseDistanceResponse))
		FATAL("MUST BE DIFFERENT");
	if(sizeof(BaseToBaseDistanceRequest)==sizeof(BaseToTagDistanceResponse))
		FATAL("MUST BE DIFFERENT");
	if(sizeof(BaseToBaseDistanceRequest)==sizeof(BaseToBaseDistanceResponse))
		FATAL("MUST BE DIFFERENT");
	if(sizeof(BaseToTagDistanceResponse)==sizeof(BaseToBaseDistanceResponse))
		FATAL("MUST BE DIFFERENT");
}

void PingManager::PrintMeasurements() {
	uprintf("bases=%d\n",(int)m_bases.size());
	for(int i=0;i!=(int)m_bases.size();i++) {
		uprintf("    id=0x%08x distance=%02f seq=%d time=%llu",m_bases[i].m_id,m_bases[i].m_distance,m_bases[i].m_sequenceNr,m_bases[i].m_lastAccessTime);
		if(m_bases[i].m_neighbors.size()) {
			if(m_bases[i].m_neighborSendIndex>=m_bases[i].m_neighbors.size())
				m_bases[i].m_neighborSendIndex=0;
			uprintf("        neighbor id=0x%08x dist16=%d\n",m_bases[i].m_neighbors[m_bases[i].m_neighborSendIndex].m_id,m_bases[i].m_neighbors[m_bases[i].m_neighborSendIndex].m_dist);
		}
	}
}

void PingManager::Sanity()const {
	for(int i=0;i!=(int)m_bases.size();i++) {
		for(int j=0;j!=(int)m_bases[i].m_neighbors.size();j++) {
			if(m_bases[i].m_neighbors[j].m_id==m_bases[i].m_id)
				FATAL("SAME!");
		}
	}
}

void PingManager::TimeoutBases() {
	Sanity();
	uint64_t time=GetTimeMicroseconds();
	for(int i=0;i<(int)m_bases.size();) {
		if(time-m_bases[i].m_lastAccessTime>10000000L) {
			UWB_NOTIFY("Timeout $%04x\n",m_bases[i].m_id);
			m_bases.remove(i);
		}else{
			++i;
		}
	}
}
void PingManager::ActiveBase::RegisterNeighbor(uint32_t id,uint16_t dist) {
	for(int j=0;j!=(int)m_neighbors.size();j++) {
		if(id==m_neighbors[j].m_id) {						// Check id match
			m_neighbors[j].m_dist=dist;						// Update distance
			return;
		}
	}
	if(!m_neighbors.remain()) {								// Check room for new entry
		return;
	}
	m_neighbors.push_back({id,dist});
}

void PingManager::RegisterAndUpdateBase(uint32_t id,float distance,uint8_t sequenceNr,uint32_t neighborId,uint16_t neighborDist, float nlos) {
	int oldestIndex=0;
	int i=0;
	if(neighborId==id) {
		FATAL("PingManager::RegisterAndUpdateBase Base id and neighbor id same $%06x!",id);
	}
	for(;i!=(int)m_bases.size();i++) {
		if(id==m_bases[i].m_id) {
			//uprintf("Update $%06x seq $%02x distance %.2f\n",id,sequenceNr,distance);
			m_bases[i].m_distance=distance;
			m_bases[i].m_sequenceNr=sequenceNr;
			m_bases[i].m_lastAccessTime=GetTimeMicroseconds();
			m_bases[i].m_frameCount=m_frameCount;
			m_bases[i].m_nlos=nlos;
			if(neighborId) {
				//uprintf("Update $%06x other $%06x dist %.d\n",id,neighborId,neighborDist);
				m_bases[i].RegisterNeighbor(neighborId,neighborDist);
			}
			return;
		}
		if(m_bases[oldestIndex].m_lastAccessTime>m_bases[i].m_lastAccessTime) {
			oldestIndex=i;
		}
	}
	if(!m_bases.remain()) {	//Overwrite oldest
		i=oldestIndex;
		UWB_NOTIFY("Overwrite $%04x\n",id);
	}
	ActiveBase ab;
	ab.m_id=id;
	ab.m_distance=distance;
	ab.m_sequenceNr=sequenceNr;
	ab.m_lastAccessTime=GetTimeMicroseconds();
	ab.m_frameCount=m_frameCount;
	ab.m_nlos=nlos;
	if(neighborId) {
		ab.RegisterNeighbor(neighborId,neighborDist);
	}
	m_bases.push_back(ab);
}


void PingManager::Begin(){
	UWB_NOTIFY("Init Ping\n");
	wrap_reset_DWIC();
	wrap_Sleep(200);
	uint32_t t0=dwt_read32bitoffsetreg(0x00,0x1c);									// Read time
	uint32_t id=dwt_wrap_readdevid();
	if(id!=DWT_C0_DEV_ID && id!=DWT_C0_PDOA_DEV_ID)									// Validate Id
		UWB_ERROR("Unable to get id from device");
	while(!dwt_wrap_checkidlerc()) {			 									// Need to make sure DW IC is in IDLE_RC before proceeding
		wrap_Sleep(1000);
		UWB_NOTIFY("waiting for dw3000 idle\n");
	}
	if(dwt_wrap_initialise(DWT_DW_INIT)==DWT_ERROR){								// Need to make sure DW IC is in IDLE_RC before proceeding
		UWB_ERROR("INIT FAILED");
	}
	dwt_wrap_setleds(DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK);							// Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
	if(dwt_wrap_configure(&config)) {												// Configure DW IC. See. If the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
		FATAL("CONFIG FAILED");
	}
	UWB_NOTIFY("dw3000 id $%08x time %d\n",id,t0);
	dwt_wrap_configuretxrf(&txconfig_options);										// Configure the TX spectrum parameters (power,PG delay and PG count)

	enable_cia();																	// enables diagnostics needed for nlos computation

	dwt_wrap_setrxantennadelay(RX_ANT_DLY);											// Apply default antenna delay value
	dwt_wrap_settxantennadelay(TX_ANT_DLY);

	dwt_wrap_setrxaftertxdelay(440);												// Set expected response's delay and timeout. As this example only handles one incoming frame with always the same delay and timeout,those values can be set here once for all.
	dwt_wrap_setrxtimeout(1000);

	dwt_wrap_setlnapamode(DWT_LNA_ENABLE|DWT_PA_ENABLE);							// Next can enable TX/RX states output on GPIOs 5 and 6 to help debug,and also TX/RX LEDs Note,in real low power applications the LEDs should not be used.

	//Clearing the SPI ready interrupt
	dwt_wrap_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);
	dwt_write32bitoffsetreg(SYS_ENABLE_LO_ID,0,DWT_INT_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR); // New value
	dwt_write32bitoffsetreg(SYS_ENABLE_HI_ID,0,DWT_ENABLE_INT);
}

void PingManager::SendTo(uint32_t targetID){
	TagToBaseDistanceRequest t2bRequest;

	t2bRequest.SetSourceId(m_id);
	t2bRequest.SetTargetId(targetID);
	t2bRequest.SetSequenceIndex(m_frameCount);

	dwt_wrap_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	dwt_wrap_writetxdata(sizeof(t2bRequest.m_data),t2bRequest.m_data,0);			// Zero offset in TX buffer.
	dwt_wrap_writetxfctrl(sizeof(t2bRequest.m_data),0,1);							// Zero offset in TX buffer,ranging.

	dwt_wrap_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);					// Start transmission,indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.

	uint32_t status_reg;
	wrap_waitforsysstatus(&status_reg,NULL,(DWT_INT_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR),0);
	dwt_wrap_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);								// Clear good RX frame event in the DW IC status register.

	if(!(status_reg&DWT_INT_RXFCG_BIT_MASK)) {
		dwt_wrap_writesysstatuslo(SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR);		// Clear RX error/timeout events in the DW IC status register.
		if(targetID)
			SetFlag(RECEIVE_TIMEOUT);
		return;
	}

	uint16_t frame_len=dwt_wrap_getframelength();									// We assume that the transmission is achieved correctly,poll for reception of a frame or error/timeout.
	if(frame_len!=sizeof(BaseToTagDistanceResponse)){
		SetFlag(PACKET_LEN);
		return;
	}

	BaseToTagDistanceResponse b2tResponse;
	dwt_wrap_readrxdata(b2tResponse.m_data,frame_len,0);							// A frame has been received,read it into the local buffer.

	if(!b2tResponse.Validate()) {
		SetFlag(PACKET_MAGIC);
		return;
	}
	uint32_t poll_tx_ts=dwt_wrap_readtxtimestamplo32();								// Retrieve poll transmission and response reception timestamps.
	uint32_t resp_rx_ts=dwt_wrap_readrxtimestamplo32();
	float nlos=nlos_prob_ipa();
	int16_t offset=dwt_wrap_readclockoffset();
	int32_t rtd_resp=b2tResponse.GetDelay();
	int16_t offsetBase=b2tResponse.GetOffset();
	rtd_resp+=DELAY_OFFSET;
	offset=(offset-offsetBase)/2;
	float clockOffsetRatio=((float)offset)/(uint32_t)(1<<26);
	int32_t rtd_init=resp_rx_ts-poll_tx_ts;
	float tof=((rtd_init-rtd_resp*(1.0f-clockOffsetRatio))/2.0f)*DWT_TIME_UNITS;	// Compute time of flight and distance,using clock offset ratio to correct for differing local and remote clock rates
	float distance=tof*SPEED_OF_LIGHT;
	uint32_t pongId=b2tResponse.GetSourceId();
	uint8_t pong_seq_nb=b2tResponse.GetSequenceIndex();
	uint16_t neighborDist=0;
	uint32_t neighborId=b2tResponse.GetNeighborDistance(&neighborDist);
	UWB_NOTIFY("id=$%06x distance %f neighbor id=$%06x dist=%d\n",pongId,distance,neighborId,neighborDist);
	RegisterAndUpdateBase(pongId,(float)distance,pong_seq_nb,neighborId,neighborDist,nlos);
}

uint32_t PingManager::BeginPing() {
	m_frameCount++;
	TimeoutBases();
	return m_bases.size();
}
void PingManager::SendPing(uint32_t index) {
	if(index>=m_bases.size())
		FATAL("SANITY");
	SendTo(m_bases[index].m_id);													// This call cannot change the number of bases
}
void PingManager::EndPing() {
	SendTo(0);																		// Broadcast to undetected bases, number bases can be increased by one
}
