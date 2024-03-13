// Based on Decawave device configuration and control functions
// Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.

#include <stdint.h>
#include <stdio.h>

#include <assert.h>
#include <string.h>
#include <thread>
#include <atomic>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/dict.h"

#include "ranging.h"

#if PLATFORM_RPI

#include "platform/gpio.h"
#include "platform/spi.h"

#include "uwb/dw_regs.h"
#include "uwb/platform_io.h"
#include "uwb/deca_spi.h"
#include "uwb/deca_dw_device_api.h"
#include "uwb/dw_device.h"

// connection pins (BROADCOM PIN NUMBERING)
const uint8_t PIN_RST = 23; // reset pin
const uint8_t PIN_IRQ = 24; // irq pin
const uint8_t PIN_CE0 = 8; // spi select pin

class RangingUWB : public Ranging {
	public:
		RangingUWB();
		virtual ~RangingUWB();
		virtual bool SendPing(uint16_t* time,float* distance);
		virtual bool Begin(const Dict& dict);
		virtual void End();
	protected:
		bool BroadcastPingLoop();
		bool InitializePing();

		std::string m_type;
		std::thread m_thread;
		std::atomic<bool> m_close;

		uint8_t m_meshId;

		uint8_t m_frame_seq_nb=0;				//Send packet index

		volatile bool m_pingInitOk=false;
		uint8_t m_initiatorId;					//Only valid in ping mode

		bool PongLoop();
};

bool RangingUWB::Begin(const Dict& dict) {
	m_close=false;
	if(!dict.Get("type",&m_type)) {
		uprintf("RangingUWB::Begin unable to get value type. Skip ranging\n");
		return false;
	}
	if(m_type!="ping" && m_type!="pong") {
		uprintf("RangingUWB::Begin type %s not valid. Skip ranging\n",m_type.c_str());
		m_type="";
		return false;
	}
	dict.Get("mesh",(char*)&m_meshId,0x12);
	dict.Get("id",(char*)&m_initiatorId,0x7f);
	if(m_type=="ping") {
		uprintf("RangingUWB:Begin type ping\n");
		RPI_GPIO::setup();
		//gpio_init();
		rpi_spi_init();
		RPI_GPIO::setup_gpio(PIN_CE0,RPI_GPIO::PIN_MODE::OUTPUT,0);
		RPI_GPIO::output_gpio(PIN_CE0,1);

		dw_irq_init(PIN_IRQ);
		deca_usleep(2000); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
		reset_DWIC(PIN_RST);
		InitializePing();
		return true;
	}
	std::atomic<bool> ready=false;
	m_thread=std::thread([&]{
		//uprintf("RangingUWB: Initialize\n");
		//uprintf("RangingUWB: Initialize GPIO\n");
		RPI_GPIO::setup();
		//gpio_init();
		//uprintf("RangingUWB: Initialize SPI\n");

		rpi_spi_init();
		RPI_GPIO::setup_gpio(PIN_CE0,RPI_GPIO::PIN_MODE::OUTPUT,0);
		RPI_GPIO::output_gpio(PIN_CE0,1);

		//uprintf("RangingUWB: Initialize IRQ\n");
		dw_irq_init(PIN_IRQ);
		deca_usleep(2000); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
		//uprintf("RangingUWB: Pin reset\n");
		reset_DWIC(PIN_RST);
		ready=true;
		uprintf("my ranging type is %s\n",m_type.c_str());
		if(m_type=="ping") {
			uprintf("RangingUWB: BroadcastPingLoop\n");
			//BroadcastPingLoop();
		}else{
			//uprintf("RangingUWB: PongLoop\n");
			PongLoop();
		}
	});
	while(!ready);
	return true;
}
void RangingUWB::End() {
}


Ranging* CreateRanging(const Dict& dict) {
	RangingUWB* r=new RangingUWB;
	r->Begin(dict);
	return r;
}
void DestroyRanging(Ranging* r) {
	RangingUWB* ru=(RangingUWB*)r;
	ru->End();
	delete r;
}

Ranging::Ranging() {
}
Ranging::~Ranging() {
}

RangingUWB::RangingUWB() : Ranging() {
}
RangingUWB::~RangingUWB() {
	if(m_type.size()) {
		m_close=true;
		uprintf("wait for ranging thread to close\n");
		m_thread.join();
		uprintf("ranging thread closed\n");
	}
}


#ifdef NDEBUG

// RESPONDER SETTING 
#define POLL_RX_TO_RESP_TX_DLY_UUS (650+200)		//Delay between frames,in UWB microseconds.

// INITIATOR SETTINGS 
#define POLL_TX_TO_RESP_RX_DLY_UUS (240+100)		//Delay between frames,in UWB microseconds.
#define RESP_RX_TIMEOUT_UUS (400+100)				// Receive response timeout.

#else

// RESPONDER SETTING 
#define POLL_RX_TO_RESP_TX_DLY_UUS (650+400)		//Delay between frames,in UWB microseconds.

// INITIATOR SETTINGS 
#define POLL_TX_TO_RESP_RX_DLY_UUS (240+200)		//Delay between frames,in UWB microseconds.
#define RESP_RX_TIMEOUT_UUS (400+200)				// Receive response timeout.


#endif




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
	(129 + 8 - 8),		// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
	DWT_STS_MODE_OFF,	// STS disabled
	DWT_STS_LEN_64,		// STS length see allowed values in Enum dwt_sts_lengths_e
	DWT_PDOA_M0			// PDOA mode off
};

// Default antenna delay values for 64 MHz PRF.
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

// Length of the common part of the message (up to and including the function code,see NOTE 3 below).
#define ALL_MSG_COMMON_LEN 10

// Index to access some of the fields in the frames involved in the process.
#define ALL_MSG_SN_IDX				2
#define RESP_MSG_POLL_RX_TS_IDX		10
#define RESP_MSG_RESP_TX_TS_IDX		14
#define RESP_MSG_TS_LEN				4

//{ 0x41,0x88,0,0xCA,0xDE,'W','A','V','I',0xE0,0,0 };

#define ALL_MSG_MESH_ID				5
#define ALL_MSG_INITIATOR_ID		6
#define RESP_MSG_LOOPBACK_IDX		7
#define RESP_MSG_TIME_U16			8

// Frame sequence number,incremented after each transmission.

// Buffer to store received messages. Its size is adjusted to longest frame that this example code is supposed to handle.
//#define RX_BUF_LEN 12 // Must be less than FRAME_LEN_MAX_EX

// Hold copy of status register state here for reference so that it can be examined at a debug breakpoint.


// Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current temperature. These values can be calibrated prior to taking reference measurements.
static dwt_txconfig_t txconfig_options={
	0x34,			// PG delay
	0xfdfdfdfd,		// TX power
	0x0				// PG count
};

bool RangingUWB::PongLoop() {
	// Configure SPI rate,DW3000 supports up to 36 MHz
	//wrap_port_set_dw_ic_spi_fastrate();
	//wrap_reset_DWIC();		// Reset and initialize DW chip. Target specific drive of RSTn line into DW3000 low for a period.

	wrap_Sleep(200);		// Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC,or could wait for SPIRDY event)

	uint32_t id=dwt_wrap_readdevid();
	//uprintf("dw3000 id 0x%08x\n",id);

	if(id!=DWT_C0_DEV_ID&&id!=DWT_C0_PDOA_DEV_ID) {
		uprintf("ERROR: Unable to get known id from deca device. id=%d\n",id);
		return false;
	}

	while(!dwt_wrap_checkidlerc()) {			 // Need to make sure DW IC is in IDLE_RC before proceeding
		wrap_Sleep(1000);
		uprintf("waiting for dw3000 idle\n");
	}

	if(dwt_wrap_initialise(DWT_DW_INIT)==DWT_ERROR) {
		FATAL("INIT FAILED");
	}

	// Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
	dwt_wrap_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	// Configure DW IC. If the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
	if(dwt_wrap_configure(&config)) {
		FATAL("CONFIG FAILED");
	}

	// Configure the TX spectrum parameters (power,PG delay and PG count)
	dwt_wrap_configuretxrf(&txconfig_options);

	// Apply default antenna delay value.
	dwt_wrap_setrxantennadelay(RX_ANT_DLY);
	dwt_wrap_settxantennadelay(TX_ANT_DLY);

	// Next can enable TX/RX states output on GPIOs 5 and 6 to help debug,and also TX/RX LEDs.
	dwt_wrap_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	// Frames used in the ranging process.
	uint8_t tx_resp_msg[]={ 0x41,0x88,0,0xCA,0xDE,'V','E','M','A',0xE1,0,0,0,0,0,0,0,0,0,0 };

	uint32_t status_reg=0;
	uint8_t rx_buffer[12];//RX_BUF_LEN];

	uprintf("RangingUWB: Enter pong loop\n");

	// Loop forever responding to ranging requests.
	while(!m_close) { // Activate reception immediately.
		dwt_wrap_rxenable(DWT_START_RX_IMMEDIATE);

		uprintf("waitforsysstatus\n");

		// Poll for reception of a frame or error/timeout.
		wrap_waitforsysstatus(&status_reg,NULL,(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR),0);

		uprintf("got sysstatus\n");

		if(status_reg&DWT_INT_RXFCG_BIT_MASK) {
			uint16_t frame_len;

			// Clear good RX frame event in the DW IC status register.
			dwt_wrap_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

			// A frame has been received,read it into the local buffer.
			frame_len=dwt_wrap_getframelength();
			if(frame_len <=sizeof(rx_buffer)) {
				dwt_wrap_readrxdata(rx_buffer,frame_len,0);
				// Check that the frame is a poll sent by "SS TWR initiator" example. As the sequence number field of the frame is not relevant,it is cleared to simplify the validation of the frame.
				if(rx_buffer[ALL_MSG_MESH_ID]==m_meshId) {
					// Retrieve poll reception timestamp.
					m_packetsReceived++;

					uint64_t poll_rx_ts=wrap_get_rx_timestamp_u64();

					// Compute response message transmission time.
					uint32_t resp_tx_time=(poll_rx_ts+(POLL_RX_TO_RESP_TX_DLY_UUS*UUS_TO_DWT_TIME))>>8;
					//uprintf("poll_rx_ts poll %llu   resp %llu\n",poll_rx_ts,resp_tx_time);
					dwt_wrap_setdelayedtrxtime(resp_tx_time);

					// Response TX timestamp is the transmission time we programmed plus the antenna delay.
					uint64_t resp_tx_ts=(((uint64_t)(resp_tx_time&0xFFFFFFFEUL))<<8)+TX_ANT_DLY;

					// Write all timestamps in the final message.
					wrap_resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX],poll_rx_ts);

					wrap_resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX],resp_tx_ts);

					// Write and send the response message.
					tx_resp_msg[ALL_MSG_SN_IDX]=m_frame_seq_nb;
					tx_resp_msg[ALL_MSG_MESH_ID]=m_meshId;
					tx_resp_msg[ALL_MSG_INITIATOR_ID]=rx_buffer[ALL_MSG_INITIATOR_ID];		//loopback initiator id
					tx_resp_msg[RESP_MSG_LOOPBACK_IDX]=rx_buffer[ALL_MSG_SN_IDX];			//loopback initiator seq nr
					*(uint16_t*)&tx_resp_msg[RESP_MSG_TIME_U16]=(uint16_t)GetTimeEpochMilliseconds();

					dwt_wrap_writetxdata(sizeof(tx_resp_msg),tx_resp_msg,0);	// Zero offset in TX buffer.
					dwt_wrap_writetxfctrl(sizeof(tx_resp_msg),0,1);				// Zero offset in TX buffer,ranging.
					int ret=dwt_wrap_starttx(DWT_START_TX_DELAYED);
					uprintf("ping received %d %d\n",ret,(int)(resp_tx_ts-poll_rx_ts));

					// If dwt_starttx() returns an error,abandon this ranging exchange and proceed to the next one.
					if(ret==DWT_SUCCESS) {
						// Poll DW IC until TX frame sent event set.
						wrap_waitforsysstatus(NULL,NULL,DWT_INT_TXFRS_BIT_MASK,0);
						// Clear TXFRS event.
						dwt_wrap_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

						// Increment frame sequence number after transmission of the poll message (modulo 256).
						m_frame_seq_nb++;
						m_packetsSend++;
					}else{
						uprintf("skip this ranging\n");
					}
				}
			}
		}else{
			// Clear RX error events in the DW IC status register.
			dwt_wrap_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
		}
	}
	return true;
}


// Inter-ranging delay period,in milliseconds.
#define RNG_DELAY_MS 1000

// Default antenna delay values for 64 MHz PRF.
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

// Length of the common part of the message (up to and including the function code).
#define ALL_MSG_COMMON_LEN 10
// Indexes to access some of the fields in the frames defined above.
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

bool RangingUWB::SendPing(uint16_t* time,float* distance) {
	if(!m_pingInitOk) {
		return false;
	}
	uint8_t rx_buffer[20];//RX_BUF_LEN];
	uint8_t tx_poll_msg[]={ 0x41,0x88,0,0xCA,0xDE,'W','A','V','I',0xE0,0,0 };
	// Frame sequence number,incremented after each transmission.
	// Write frame data to DW IC and prepare transmission.
	tx_poll_msg[ALL_MSG_SN_IDX]=m_frame_seq_nb;
	tx_poll_msg[ALL_MSG_MESH_ID]=m_meshId;
	tx_poll_msg[ALL_MSG_INITIATOR_ID]=m_initiatorId;
	dwt_wrap_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	dwt_wrap_writetxdata(sizeof(tx_poll_msg),tx_poll_msg,0);	// Zero offset in TX buffer.
	dwt_wrap_writetxfctrl(sizeof(tx_poll_msg),0,1);				// Zero offset in TX buffer,ranging.

	m_frame_seq_nb++;
	m_packetsSend++;

	// Start transmission,indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
	dwt_wrap_starttx(DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED);

	// We assume that the transmission is achieved correctly,poll for reception of a frame or error/timeout.
	uint32_t status_reg;
	wrap_waitforsysstatus(&status_reg,NULL,(DWT_INT_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR),0);

	// Increment frame sequence number after transmission of the poll message (modulo 256).

	if(status_reg&DWT_INT_RXFCG_BIT_MASK) {
		uint16_t frame_len;
		// Clear good RX frame event in the DW IC status register.
		dwt_wrap_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
		// A frame has been received,read it into the local buffer.
		frame_len=dwt_wrap_getframelength();
		if(frame_len<=sizeof(rx_buffer)){
			dwt_wrap_readrxdata(rx_buffer,frame_len,0);
			// Check that the frame is for this mesh and request is send from this initiator
			if(rx_buffer[ALL_MSG_MESH_ID]==m_meshId && rx_buffer[ALL_MSG_INITIATOR_ID]==m_initiatorId) {
				// Retrieve poll transmission and response reception timestamps.
				uint32_t poll_tx_ts=dwt_wrap_readtxtimestamplo32();
				uint32_t resp_rx_ts=dwt_wrap_readrxtimestamplo32();
				// Read carrier integrator value and calculate clock offset ratio.
				float clockOffsetRatio=((float)dwt_wrap_readclockoffset())/(uint32_t)(1<<26);
				// Get timestamps embedded in response message.
				uint32_t poll_rx_ts,resp_tx_ts;
				wrap_resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX],&poll_rx_ts);
				wrap_resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX],&resp_tx_ts);
				// Compute time of flight and distance,using clock offset ratio to correct for differing local and remote clock rates
				int32_t rtd_init=resp_rx_ts-poll_tx_ts;
				int32_t rtd_resp=resp_tx_ts-poll_rx_ts;
				double tof=((rtd_init-rtd_resp*(1-clockOffsetRatio))/2.0)*DWT_TIME_UNITS;
				double dist=tof*SPEED_OF_LIGHT;
				uint16_t tx_time_us=*(uint16_t*)&rx_buffer[RESP_MSG_TIME_U16];
				*distance=dist;
				*time=tx_time_us;
				m_packetsReceived++;
				uprintf("%d %d %d %3.4f\n",m_initiatorId,tx_time_us,rx_buffer[RESP_MSG_LOOPBACK_IDX],dist);
				uprintf("response time 0x%08x seq loopback %d distance: %3.4f m\n",resp_rx_ts,rx_buffer[RESP_MSG_LOOPBACK_IDX],dist);
			}
		}
	}else{
		*distance=0;
		*time=0;
		// Clear RX error/timeout events in the DW IC status register.
		dwt_wrap_writesysstatuslo(SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR);
	}
	return true;
}
bool RangingUWB::InitializePing() {
	// Configure SPI rate,DW3000 supports up to 36 MHz
	wrap_Sleep(2000); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC,or could wait for SPIRDY event)

	uint32_t id=dwt_wrap_readdevid();
	//uprintf("dw3000 id 0x%08x\n",id);

	if(id!=DWT_C0_DEV_ID&&id!=DWT_C0_PDOA_DEV_ID) {
		uprintf("ERROR: Unable to get known id from deca device. id=%d\n",id);
		return false;
	}

	while(!dwt_wrap_checkidlerc()) {			 // Need to make sure DW IC is in IDLE_RC before proceeding
		wrap_Sleep(1000);
		uprintf("waiting for dw3000 idle\n");
	}

	if(dwt_wrap_initialise(DWT_DW_INIT)==DWT_ERROR){
		return false;
	}

	// Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
	dwt_wrap_setleds(DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK);

	// Configure DW IC. See. If the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
	if(dwt_wrap_configure(&config)) {
		FATAL("CONFIG FAILED");
	}
	// Configure the TX spectrum parameters (power,PG delay and PG count)
	dwt_wrap_configuretxrf(&txconfig_options);

	// Apply default antenna delay value
	dwt_wrap_setrxantennadelay(RX_ANT_DLY);
	dwt_wrap_settxantennadelay(TX_ANT_DLY);

	// Set expected response's delay and timeout. As this example only handles one incoming frame with always the same delay and timeout,those values can be set here once for all.
	dwt_wrap_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_wrap_setrxtimeout(RESP_RX_TIMEOUT_UUS);

	// Next can enable TX/RX states output on GPIOs 5 and 6 to help debug,and also TX/RX LEDs Note,in real low power applications the LEDs should not be used.
	dwt_wrap_setlnapamode(DWT_LNA_ENABLE|DWT_PA_ENABLE);
	m_pingInitOk=true;
	return true;
}
bool RangingUWB::BroadcastPingLoop() {
	if(!InitializePing())
		FATAL("Ranging InitializePing FAILED");
	uprintf("Ping loop forever\n");
	// Loop forever initiating ranging exchanges.
	while (1) {
		float distance;
		uint16_t time;
		if(SendPing(&time,&distance)) {
			uprintf("time %d distance %.2fm\n",time,distance);
		}
		wrap_Sleep(RNG_DELAY_MS);
	}
	return true;
}
#else//PLATFORM_RPI

Ranging* CreateRanging(const Dict& dict) {
	return 0;
}
void DestroyRanging(Ranging* r) {
	delete r;
}

#endif//PLATFORM_RPI
