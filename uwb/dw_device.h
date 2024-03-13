#ifndef _DW_DEVICE_H_
#define _DW_DEVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

//extern void EnableDebugPrintSPI();
//extern void DisableDebugPrintSPI();
//void PrintBuffer(const char* name,const uint8_t *dataBuffer,uint16_t dataLength);
//extern void PrintBuffer(const char* name,const uint8_t *dataBuffer,uint16_t dataLength);

#define SPEED_OF_LIGHT			(299702547)

#define DW3000_SPI_FAC			(0<<6|1<<0)
#define DW3000_SPI_FARW			(0<<6|0<<0)
#define DW3000_SPI_EAMRW		(1<<6)

//DW3000 OTP operating parameter set selection
#define DWT_OPSET_LONG			(0x0<<11)
#define DWT_OPSET_SCP			(0x1<<11)
#define DWT_OPSET_SHORT			(0x2<<11)

#define DWT_C0_PDOA_DEV_ID		(0xDECA0312)  //!< DW3000 MPW C0 (with PDOA) silicon device ID
#define DWT_C0_DEV_ID			(0xDECA0302)  //!< DW3000 MPW C0 (non PDOA) silicon device ID

#define LDOTUNELO_ADDRESS		(0x04)
#define LDOTUNEHI_ADDRESS		(0x05)
#define PARTID_ADDRESS			(0x06)
#define BIAS_TUNE_ADDRESS		(0xA)
#define DGC_TUNE_ADDRESS		(0x20)
#define LOTID_ADDRESS			(0x07)
#define VBAT_ADDRESS			(0x08)
#define VTEMP_ADDRESS			(0x09)
#define VTEMP_ADDRESS			(0x09)
#define OTPREV_ADDRESS			(0x1F)
#define XTRIM_ADDRESS			(0x1E)

#define DWT_DGC_CFG0			0x10000240

// LDO and BIAS tune kick
#define LDO_BIAS_KICK			(0x180) // Writing to bit 7 and 8

#define FORCE_CLK_SYS_TX		(1)
#define FORCE_CLK_AUTO			(5)

//SYSCLK
#define FORCE_SYSCLK_PLL		(2)
#define FORCE_SYSCLK_FOSCDIV4	(1)
#define FORCE_SYSCLK_FOSC		(3)
//RX and TX CLK
#define FORCE_CLK_PLL			(2)
#define DWT_AUTO_CLKS			(0x200 | 0x200000 | 0x100000) //this is the default value of CLK_CTRL register

typedef enum {
	CH5_DGC_LUT_0=0x1c0fd,
	CH5_DGC_LUT_1=0x1c43e,
	CH5_DGC_LUT_2=0x1c6be,
	CH5_DGC_LUT_3=0x1c77e,
	CH5_DGC_LUT_4=0x1cf36,
	CH5_DGC_LUT_5=0x1cfb5,
	CH5_DGC_LUT_6=0x1cff5
} dwt_configmrxlut_ch5_e;

typedef enum {
	CH9_DGC_LUT_0=0x2a8fe,
	CH9_DGC_LUT_1=0x2ac36,
	CH9_DGC_LUT_2=0x2a5fe,
	CH9_DGC_LUT_3=0x2af3e,
	CH9_DGC_LUT_4=0x2af7d,
	CH9_DGC_LUT_5=0x2afb5,
	CH9_DGC_LUT_6=0x2afb5
} dwt_configmrxlut_ch9_e;

#define DGC_CFG1_ID				0x30020
//#define DGC_CFG1_LEN			(4U)
//#define DGC_CFG1_MASK			0xFFFFFFFFUL
#define DWT_DGC_CFG1			0x1b6da489

#define STSQUAL_THRESH_64		(0.90f)
#define STS_LEN_SUPPORTED		7 // The supported STS length options

#define dwt_or8bitoffsetreg(addr,offset,or_val) dwt_modify8bitoffsetreg(addr,offset,-1,or_val)
#define dwt_or16bitoffsetreg(addr,offset,or_val) dwt_modify16bitoffsetreg(addr,offset,-1,or_val)
#define dwt_and_or16bitoffsetreg(addr,offset,and_val,or_val) dwt_modify16bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_or32bitoffsetreg(addr,offset,or_val) dwt_modify32bitoffsetreg(addr,offset,-1,or_val)
#define dwt_write32bitreg(addr,value) dwt_write32bitoffsetreg(addr,0,value)
#define dwt_read32bitreg(addr) dwt_read32bitoffsetreg(addr,0)
#define dwt_and32bitoffsetreg(addr,offset,and_val) dwt_modify32bitoffsetreg(addr,offset,and_val,0)
#define dwt_and16bitoffsetreg(addr,offset,and_val) dwt_modify16bitoffsetreg(addr,offset,and_val,0)
#define dwt_writefastCMD(cmd) dwt3000_writetodevice(cmd,0,0,0)
#define dwt_and8bitoffsetreg(addr,offset,and_val) dwt_modify8bitoffsetreg(addr,offset,and_val,0)

void dwt3000_readfromdevice(uint32_t regFileID,uint16_t index,uint16_t length,uint8_t *buffer);
void dwt3000_writetodevice(uint32_t regFileID,uint16_t index,uint16_t length,uint8_t *buffer);
uint8_t dwt_read8bitoffsetreg(uint32_t regFileID,uint16_t regOffset);
void dwt_modify16bitoffsetreg(const uint32_t regFileID,const uint16_t regOffset,const uint16_t _and,const uint16_t _or);
void dwt_modify8bitoffsetreg(const uint32_t regFileID,const uint16_t regOffset,const uint8_t _and,const uint8_t _or);
uint32_t dwt_wrap_readdevid(void);
int dwt_wrap_check_dev_id(void);
void _dwt_prog_ldo_and_bias_tune(void);
uint32_t _dwt_otpread(uint16_t address);
void dwt_write16bitoffsetreg(uint32_t regFileID,uint16_t regOffset,uint16_t regval);
uint32_t dwt_read32bitoffsetreg(uint32_t regFileID,uint16_t regOffset);
int dwt_wrap_initialise_old(uint8_t mode);
int dwt_wrap_initialise(uint8_t mode);
void dwt_wrap_setleds(uint8_t mode);
void dwt_write8bitoffsetreg(uint32_t regFileID,uint16_t regOffset,uint8_t regval);
uint16_t get_sts_mnth (uint16_t cipher,uint8_t threshold,uint8_t shift_val);
void dwt_write32bitoffsetreg(uint32_t regFileID,uint16_t regOffset,uint32_t regval);
void _dwt_kick_dgc_on_wakeup(int8_t channel);
void dwt_modify32bitoffsetreg(const uint32_t regFileID,const uint16_t regOffset,const uint32_t _and,const uint32_t _or);

void dwt_wrap_setplenfine(uint8_t preambleLength);
void dwt_wrap_setdwstate_old(uint8_t state);
void dwt_wrap_setdwstate(uint8_t state);
void dwt_wrap_force_clocks(int clocks);

void dwt_wrap_configmrxlut(uint8_t channel);
int dwt_wrap_run_pgfcal(void);
int dwt_wrap_pgf_cal(uint8_t ldoen);
int dwt_wrap_configure_old(dwt_config_t* config);
int dwt_wrap_configure(dwt_config_t* config);

void dwt_wrap_configuretxrf(dwt_txconfig_t* config);
void dwt_wrap_setrxantennadelay(uint16_t antennaDly);
void dwt_wrap_setrxaftertxdelay(uint32_t rxDelayTime);
void dwt_wrap_setrxtimeout(uint32_t time);
void dwt_wrap_setlnapamode(uint8_t lna_pa);
void dwt_wrap_writesysstatuslo(uint32_t mask);
int dwt_wrap_writetxdata(uint16_t txDataLength,uint8_t* txDataBytes,uint16_t txBufferOffset);
void dwt_wrap_writetxfctrl(uint16_t txFrameLength,uint16_t txBufferOffset,uint8_t ranging);
int dwt_wrap_starttx(uint8_t mode);
void dwt_wrap_readrxdata(uint8_t* buffer,uint16_t length,uint16_t rxBufferOffset);
uint32_t dwt_wrap_readtxtimestamplo32(void);
uint16_t dwt_wrap_getframelength(void);
void dwt_wrap_settxantennadelay(uint16_t txDelay);
uint32_t dwt_wrap_readrxtimestamplo32(void);
int16_t dwt_wrap_readclockoffset(void);
uint32_t dwt_wrap_readsysstatuslo(void);
uint32_t dwt_wrap_readsysstatushi(void);
void wrap_waitforsysstatus(uint32_t* lo_result,uint32_t* hi_result,uint32_t lo_mask,uint32_t hi_mask);
uint8_t dwt_wrap_checkidlerc();
void wrap_resp_msg_get_ts(uint8_t* ts_field,uint32_t* ts);
uint64_t wrap_get_rx_timestamp_u64();
void dwt_wrap_setdelayedtrxtime(uint32_t starttime);
void wrap_resp_msg_set_ts(uint8_t*ts_field,const uint64_t ts);

//Transmit PONG
int dwt_wrap_rxenable(int mode);

void enable_cia();
float nlos_prob_ipa();

// UWB microsecond (uus) to device time unit (dtu,around 15.65 ps) conversion factor. 1 uus=512 / 499.2 µs and 1 µs=499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 63898

#ifdef __cplusplus
}
#endif

#endif//_DW_DEVICE_H_
