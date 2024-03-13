
#include <stdio.h>
#include "assert.h"
#include "string.h"
#include <stdarg.h>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/dict.h"

#if PLATFORM_RPI

#include <deca_dw_device_api.h>
#include "uwb/platform_io.h"
#include "uwb/dw_device.h"
#include "uwb/dw_regs.h"

#include "uwb.h"

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


void PingManager::GetMeasurements(Measurements* measurements) {
	uint32_t statusMask=m_status.GetStatusMask();
	int pos=0;
	for(int i=0;i!=STATUS_FLAG_MAX;i++) {
		if(statusMask&(1<<i)) {
			const char* str=StatusFlagToName((eStatusFlag)i);
			if(pos+strlen(str)+2<MAX_STATUS_STRING_LENGTH)
				pos+=sprintf(&measurements->m_status[pos],"%s,",str);
		}
	}
	measurements->m_status[pos?pos-1:0]=0;
	measurements->m_frameCount=m_frameCount;
	for(int i=0;i!=(int)m_bases.size();i++) {
		Measurements::Base base;
		base.m_sequenceNr=m_bases[i].m_sequenceNr;
		base.m_time=m_bases[i].m_lastAccessTime;
		base.m_frameCount=m_bases[i].m_frameCount;
		base.m_id=m_bases[i].m_id;
		base.m_distance=m_bases[i].m_distance;
		base.m_nlos=m_bases[i].m_nlos;
		if(m_bases[i].m_neighbors.size()) {
			if(m_bases[i].m_neighborSendIndex>=m_bases[i].m_neighbors.size())
				m_bases[i].m_neighborSendIndex=0;
			Measurements::Base::Neighbor neighbor;
			neighbor.m_id=m_bases[i].m_neighbors[m_bases[i].m_neighborSendIndex].m_id;
			neighbor.m_dist=m_bases[i].m_neighbors[m_bases[i].m_neighborSendIndex].m_dist;
			base.m_neighbors.push_back(neighbor);
#if 1
			m_bases[i].m_neighbors.remove(m_bases[i].m_neighborSendIndex);
#else
			m_bases[i].m_neighborSendIndex++;
#endif
		}
		measurements->m_bases.push_back(base);
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
	wrap_port_set_dw_ic_spi_fastrate();
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
	//uprintf("rtd_resp %d\n",rtd_resp);
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

//PongManager
PongManager::PongManager() {
	m_id=GetUniqueId24();
	m_frameCount=0;
	ResetFrameTags();
}

void PongManager::ResetFrameTags() {
}
void PongManager::RegisterFrameTag(uint32_t id,uint8_t seqNr) {
	for(uint32_t i=0;i!=m_tags.size();i++) {
		if(m_tags[i].m_id==id) {
			m_tags[i].m_seqNr=seqNr;
			return;
		}
	}
	if(!m_tags.remain())
		FATAL("Too many tags");
	FrameTags ft;
	ft.m_id=id;
	ft.m_lastDataIndex=0;
	ft.m_neighborSendIndex=0;
	ft.m_seqNr=seqNr;
	m_tags.push_back(ft);
}
bool PongManager::CheckFrameTag(uint32_t id,uint8_t seqNr)const {
	for(uint32_t i=0;i!=m_tags.size();i++) {
		if(m_tags[i].m_id==id && m_tags[i].m_seqNr==seqNr) {
			return true;
		}
	}
	return false;
}

void PongManager::Begin() {
	//UWB_NOTIFY("Begin Pong\n");
	wrap_reset_DWIC();
	wrap_Sleep(200);
	uint32_t t0=dwt_read32bitoffsetreg(0x00,0x1c);									// Read systime
	uint32_t id=dwt_wrap_readdevid();
	if(id!=DWT_C0_DEV_ID && id!=DWT_C0_PDOA_DEV_ID)
		UWB_ERROR("Unable to get id from device");
	while(!dwt_wrap_checkidlerc()) {			 									// Need to make sure DW IC is in IDLE_RC before proceeding
		wrap_Sleep(1000);
		UWB_NOTIFY("waiting for dw3000 idle\n");
	}
	if(dwt_wrap_initialise(DWT_DW_INIT)==DWT_ERROR) {
		UWB_ERROR("INIT FAILED");
	}
	dwt_wrap_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);						// Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
	if(dwt_wrap_configure(&config)) {												// Configure DW IC. If the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
		UWB_ERROR("CONFIG FAILED");
	}
	UWB_NOTIFY("dw3000 id $%08x time %d\n",id,t0);
	dwt_wrap_configuretxrf(&txconfig_options);										// Configure the TX spectrum parameters (power,PG delay and PG count)
	dwt_wrap_setrxantennadelay(RX_ANT_DLY);											// Apply default antenna delay value
	dwt_wrap_settxantennadelay(TX_ANT_DLY);
	dwt_wrap_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);							// Next can enable TX/RX states output on GPIOs 5 and 6 to help debug,and also TX/RX LEDs.
}

void PongManager::ReceiveData(const uint8_t* first64) {
	uint32_t tagId=((uint32_t)first64[0]<<16)|((uint32_t)first64[1]<<8)|first64[2];
	dwt_wrap_rxenable(DWT_START_RX_IMMEDIATE);
	uint32_t status_reg=0;															// Poll for reception of a frame or error/timeout.
	wrap_waitforsysstatus(&status_reg,NULL,(DWT_INT_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR),0);
	if(!(status_reg&DWT_INT_RXFCG_BIT_MASK)) {										// Clear RX error events in the DW IC status register.
		dwt_wrap_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
		//StatusFlags::Instance().SetFlag(RECEIVE_FAIL_DATA);
		return;
	}
	dwt_wrap_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);								// Clear good RX frame event in the DW IC status register.
	uint16_t frame_len1=dwt_wrap_getframelength();
	uint8_t testdata[64];
	if(frame_len1!=sizeof(testdata)) {
		//StatusFlags::Instance().SetFlag(RECEIVE_FAIL_DATA);
		return;
	}
	dwt_wrap_readrxdata(testdata,frame_len1,0);										// A frame has been received,read it into the local buffer.
	uint8_t ix=testdata[0];
	for(uint32_t i=0;i!=m_tags.size();i++) {
		if(m_tags[i].m_id==tagId) {
			if(m_tags[i].m_lastDataIndex+1!=ix)
				uprintf("missing data $%02x != $%02x from tag $%06x\n",ix,m_tags[i].m_lastDataIndex,tagId);
	       m_tags[i].m_lastDataIndex=ix;
		}
	}
	m_dataReceivedBytesize+=128;
	//uprintf("received test data1 length %d first bytes $%02x,$%02x,$%02x,$%02x\n",frame_len,testdata[0],testdata[1],testdata[2],testdata[3]);
	//uprintf("received test data\n");
}

void PongManager::PrintMesh() {
	uprintf("Number other bases %d\n",m_neighbors.size());
	for(uint32_t i=0;i!=m_neighbors.size();i++) {
		uprintf("id=$%08x dist=%d\n",(int)m_neighbors[i].m_id,(int)m_neighbors[i].m_dist);
	}
}

void PongManager::RegisterNeighbor(uint32_t baseId) {
	for(uint32_t i=0;i!=m_neighbors.size();i++) {
		if(m_neighbors[i].m_id==baseId) {
			return;
		}
	}
	if(!m_neighbors.remain()) {
		uprintf("No room for base $%08x\n",baseId);
		UWB_ERROR("NO ROOM");
		return;
	}
	m_neighbors.push_back({baseId,0,0});
	//uprintf("base $%08x register other base $%08x\n",m_id,baseId);
}
void PongManager::UnregisterNeighbor(uint32_t baseId) {
	for(uint32_t i=0;i!=m_neighbors.size();i++) {
		if(m_neighbors[i].m_id==baseId) {					// Is it the correct id
			m_neighbors.remove(i);
			if(m_neighborMeasureIndex>=m_neighbors.size())	// Ensure valid measure index
				m_neighborMeasureIndex=0;
			return;
		}
	}
	uprintf("Unable to unregister base $%08x\n",baseId);
}


bool PongManager::ReadNextNeighborDistance(uint32_t* id,uint16_t* dist,uint32_t tagId) {
	if(!m_neighbors.size()) {
		return false;
	}
	for(uint32_t i=0;i!=m_tags.size();i++) {
		if(m_tags[i].m_id==tagId) {
			for(uint32_t j=0;j!=m_neighbors.size();j++) {
				if(m_tags[i].m_neighborSendIndex>=m_neighbors.size())
					m_tags[i].m_neighborSendIndex=0;
				*dist=m_neighbors[m_tags[i].m_neighborSendIndex].m_dist;
				*id=m_neighbors[m_tags[i].m_neighborSendIndex].m_id;
				m_tags[i].m_neighborSendIndex++;
				if(*dist)																// Check if last measurment valid
					return true;
			}
			return false;
		}
	}
	return false;
}

#define MAX_CONTINUOUS_FAIL 2

void PongManager::MeasureDistanceToNeighbor() {
	if(!m_neighbors.size()) {
		return;
	}
	if(m_neighborMeasureIndex>=m_neighbors.size()) {
		m_neighborMeasureIndex=0;
	}
	Neighbor* ob=&m_neighbors[m_neighborMeasureIndex];
	uint32_t neighborId=ob->m_id;
	ob->m_dist=0;
	m_neighborMeasureIndex++;

	wrap_Sleep(10);																	// To avoid dw lock

	dwt_wrap_setrxaftertxdelay(400);												// Set expected response's delay and timeout.
	dwt_wrap_setrxtimeout(800);

	BaseToBaseDistanceRequest b2bRequest;
	b2bRequest.SetSourceId(m_id);
	b2bRequest.SetTargetId(neighborId);
	b2bRequest.SetSequenceIndex(m_frameCount);

	dwt_wrap_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

	dwt_wrap_writetxdata(sizeof(b2bRequest.m_data),b2bRequest.m_data,0);
	dwt_wrap_writetxfctrl(sizeof(b2bRequest.m_data),0,1);
	dwt_wrap_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);					// Start transmission and wait for response

	uint32_t status_reg;
	wrap_waitforsysstatus(&status_reg,NULL,(DWT_INT_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR),0);
	dwt_wrap_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);								// Clear good RX frame event in the DW IC status register.

	if(!(status_reg&DWT_INT_RXFCG_BIT_MASK)) {
		if(ob->m_continuousFailCount++>MAX_CONTINUOUS_FAIL) {
			UnregisterNeighbor(neighborId);
		}
		dwt_wrap_writesysstatuslo(SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR);		// Clear RX error/timeout events in the DW IC status register.
		UWB_NOTIFY("PongManager::MeasureDistanceToNeighbor from $%06x to $%06x failed\n",m_id,neighborId);
		return;
	}
	uint16_t frame_len=dwt_wrap_getframelength();									// get length of received data
	BaseToBaseDistanceResponse b2bResponse;
	if(frame_len!=sizeof(b2bResponse)){
		if(ob->m_continuousFailCount++>MAX_CONTINUOUS_FAIL) {
			UnregisterNeighbor(neighborId);
		}
		return;
	}
	dwt_wrap_readrxdata(b2bResponse.m_data,frame_len,0);								// Read received data
	if(!b2bResponse.Validate()) {
		if(ob->m_continuousFailCount++>MAX_CONTINUOUS_FAIL) {
			UnregisterNeighbor(neighborId);
		}
		return;
	}
	uint32_t poll_tx_ts=dwt_wrap_readtxtimestamplo32();								// Retrieve poll transmission and response reception timestamps.
	uint32_t resp_rx_ts=dwt_wrap_readrxtimestamplo32();
	int16_t offset=dwt_wrap_readclockoffset();
	int32_t rtd_resp=b2bResponse.GetDelay();
	int16_t offsetBase=b2bResponse.GetOffset();
	rtd_resp+=DELAY_OFFSET;
	offset=(offset-offsetBase)/2;
	float clockOffsetRatio=((float)offset)/(uint32_t)(1<<26);
	int32_t rtd_init=resp_rx_ts-poll_tx_ts;
	float tof=((rtd_init-rtd_resp*(1.0f-clockOffsetRatio))/2.0f)*DWT_TIME_UNITS;	// Compute time of flight and distance,using clock offset ratio to correct for differing local and remote clock rates
	float distance=tof*SPEED_OF_LIGHT;
	distance=CLAMP(1.0f,distance*100.0f,10000.0f);
	ob->m_dist=(uint16_t)distance;
	ob->m_continuousFailCount=0;														// Reset fail counter
	UWB_NOTIFY("MeasureDistanceToNeighbor End dist u16 %d source id $%08x\n",ob->m_dist,neighborId);
}

void PongManager::Receive() {
	dwt_wrap_setrxaftertxdelay(0);														// Set expected response's delay and timeout. As this example only handles one incoming frame with always the same delay and timeout,those values can be set here once for all.
	dwt_wrap_setrxtimeout(0);
	dwt_wrap_rxenable(DWT_START_RX_IMMEDIATE);
	uint32_t status_reg=0;																// Poll for reception of a frame or error/timeout.
	wrap_waitforsysstatus(&status_reg,NULL,(DWT_INT_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_ERR),0);
	if(!(status_reg&DWT_INT_RXFCG_BIT_MASK)) {											// Clear RX error events in the DW IC status register.
		dwt_wrap_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
		//StatusFlags::Instance().SetFlag(RECEIVE_FAIL);
		return;
	}
	dwt_wrap_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);									// Clear good RX frame event in the DW IC status register.
	uint16_t frame_len=dwt_wrap_getframelength();
	if(frame_len==sizeof(TagToBaseDistanceRequest)) {
		TagToBaseDistanceRequest t2bRequest;
		dwt_wrap_readrxdata(t2bRequest.m_data,frame_len,0);								// A frame has been received,read it into the local buffer.
		if(!t2bRequest.Validate())
			return;
		uint32_t targetId=t2bRequest.GetTargetId();
		//uint64_t t0=GetTimeMicroseconds();
		if(targetId && targetId!=m_id) {												// Not for me?
			return;
		}
		uint32_t tagId=t2bRequest.GetSourceId();										// Tag identifier
		uint8_t seqNr=t2bRequest.GetSequenceIndex();
		if(!targetId){																	// Broadcast ping?
			if(CheckFrameTag(tagId,seqNr)) {											// Allready received ping specifically for me?
				//UWB_NOTIFY("Broadcast frame allready received $%02x\n",m_lastReceivedSeqNr);
				return;
			}
			//uprintf("Received broadcast $%06x\n",tagId);
		}else{
			RegisterFrameTag(tagId,seqNr);
			//UWB_NOTIFY("Register frame for me $%02x\n",m_lastReceivedSeqNr);
		}
		uint32_t offset=dwt_wrap_readclockoffset();
		uint64_t poll_rx_ts=wrap_get_rx_timestamp_u64();								// Retrieve poll reception timestamp.

		uint32_t resp_tx_time=(poll_rx_ts+(550*UUS_TO_DWT_TIME))>>8;					// Compute response message transmission time in UWB microseconds.

		if(m_id==0x61bd3a) {
			resp_tx_time=(poll_rx_ts+(850*UUS_TO_DWT_TIME))>>8;						// Compute response message transmission time in UWB microseconds.
		}else{
		}

		dwt_wrap_setdelayedtrxtime(resp_tx_time);										// Response TX timestamp is the transmission time we programmed plus the antenna delay.
		uint64_t resp_tx_ts=(((uint64_t)(resp_tx_time&0xFFFFFFFEUL))<<8)+TX_ANT_DLY;
		int32_t delay=resp_tx_ts-poll_rx_ts;
		BaseToTagDistanceResponse b2tResponse;
		b2tResponse.SetDelay(delay-DELAY_OFFSET);
		b2tResponse.SetOffset(offset);
		b2tResponse.SetSequenceIndex(m_frameCount);										// Write and send the response message.
		b2tResponse.SetSourceId(m_id);
		uint16_t neighborDist;
		uint32_t neighborId;
		if(ReadNextNeighborDistance(&neighborId,&neighborDist,tagId)) {
			b2tResponse.SetNeighborDistance(neighborId,neighborDist);
		}
		dwt_wrap_writetxdata(sizeof(b2tResponse.m_data),b2tResponse.m_data,0);			// Zero offset in TX buffer.
		dwt_wrap_writetxfctrl(sizeof(b2tResponse.m_data),0,1);							// Zero offset in TX buffer,ranging.
		//uint64_t t1=GetTimeMicroseconds();
		int ret=dwt_wrap_starttx(DWT_START_TX_DELAYED);									// If dwt_starttx() returns an error,abandon this ranging exchange and proceed to the next one
		if(ret==DWT_SUCCESS) {
			wrap_waitforsysstatus(NULL,NULL,DWT_INT_TXFRS_BIT_MASK,0);					// Poll DW IC until TX frame sent event set
			dwt_wrap_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);							// Clear TXFRS event.
			m_frameCount++;
		}else{
			//StatusFlags::Instance().SetFlag(TX_FAIL);
		}
		//uprintf("base $%06x send response to tag $%06x\n",m_id,tagId);
		uint64_t time=GetTimeMicroseconds();
		if(time>m_lastNeighborPingTime) {
			//uprintf("Measure! $%06x\n",m_id);
			if(m_lastNeighborPingTime) {
				MeasureDistanceToNeighbor();
			}
			float r=RandomUnitFloat();
			m_lastNeighborPingTime=time+((uint64_t)(r*5000000))+5000000L;
		}
		deca_usleep(1000);
		//UWB_NOTIFY("Respond to broadcast since no ping for me this frame. last received frame $%02x does not match broadcast frame $%02x\n",m_lastReceivedSeqNr,frameIndex);
		return;
	}
	if(frame_len==sizeof(BaseToBaseDistanceRequest)) {
		BaseToBaseDistanceRequest b2bRequest;
		dwt_wrap_readrxdata(b2bRequest.m_data,frame_len,0);								// A frame has been received,read it into the local buffer.
		if(!b2bRequest.Validate()) {
			return;
		}
		uint32_t targetId=b2bRequest.GetTargetId();
		if(targetId && targetId!=m_id) {												// Not for me?
			return;
		}

		BaseToBaseDistanceResponse b2bResponse;
		uint32_t offset=dwt_wrap_readclockoffset();
		uint64_t poll_rx_ts=wrap_get_rx_timestamp_u64();								// Retrieve poll reception timestamp.
		uint32_t resp_tx_time=(poll_rx_ts+(550*UUS_TO_DWT_TIME))>>8;					// Compute response message transmission time in UWB microseconds.
		dwt_wrap_setdelayedtrxtime(resp_tx_time);										// Response TX timestamp is the transmission time we programmed plus the antenna delay.
		uint64_t resp_tx_ts=(((uint64_t)(resp_tx_time&0xFFFFFFFEUL))<<8)+TX_ANT_DLY;
		int32_t delay=resp_tx_ts-poll_rx_ts;
		b2bResponse.SetDelay(delay-DELAY_OFFSET);
		b2bResponse.SetOffset(offset);
		b2bResponse.SetSequenceIndex(m_frameCount);										// Write and send the response message.
		b2bResponse.SetSourceId(m_id);
		dwt_wrap_writetxdata(sizeof(b2bResponse.m_data),b2bResponse.m_data,0);			// Zero offset in TX buffer.
		dwt_wrap_writetxfctrl(sizeof(b2bResponse.m_data),0,1);							// Zero offset in TX buffer,ranging.
		int ret=dwt_wrap_starttx(DWT_START_TX_DELAYED);									// If dwt_starttx() returns an error,abandon this ranging exchange and proceed to the next one
		if(ret==DWT_SUCCESS) {
			wrap_waitforsysstatus(NULL,NULL,DWT_INT_TXFRS_BIT_MASK,0);					// Poll DW IC until TX frame sent event set
			dwt_wrap_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);							// Clear TXFRS event.
		}
		UWB_NOTIFY("Received ping from other base $%06x\n",b2bRequest.GetSourceId());
		deca_usleep(1000);
		return;
	}
	if(frame_len==sizeof(BaseToTagDistanceResponse)) {									// Check response from another base
		BaseToTagDistanceResponse b2tResponse;
		dwt_wrap_readrxdata(b2tResponse.m_data,frame_len,0);
		if(!b2tResponse.Validate())
			return;
		uint32_t baseId=b2tResponse.GetSourceId();
		RegisterNeighbor(baseId);														// Register this other base
		UWB_NOTIFY("base $%08x received base pong from $%08x %d\n",m_id,baseId,frame_len);
		return;
	}
	uint8_t testdata[64];
	if(frame_len==sizeof(testdata)) {													// Check test data
		dwt_wrap_readrxdata(testdata,frame_len,0);										// Receive test data
		ReceiveData(testdata);
		return;
	}
}

#endif//PLATFORM_RPI
