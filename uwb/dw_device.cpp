// Based on Decawave device configuration and control functions
// Copyright 2013-2020 (c) Decawave Ltd,Dublin,Ireland.

#include <stdint.h>
#include <stdio.h>

#include <deca_dw_device_api.h>
#include <assert.h>
#include <string.h>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"

#include "platform_io.h"
#include "dw_device.h"
#include "dw_regs.h"

#include "platform/spi.h"

#define CIA_MANUALLOWERBOUND_TH_64 (0x10) //cia lower bound threshold values for 64 MHz PRF

#define INDIRECT_POINTER_A_ID	0x1D0000	// pointer to access indirect access buffer A
#define INDIRECT_POINTER_B_ID	0x1E0000	// pointer to access indirect access buffer B

#define BUF0_RX_TIME			0x180004	// part of min set (RX time ~ RX_TIME_O)

#define BUF1_RX_FINFO			0x1800E8	// part of min set
#define BUF1_RX_TIME			0x1800EC	// part of min set (RX time ~ RX_TIME_O)
#define BUF1_RX_TIME1			0x1800F0	// part of min set

#define RX_TIME_RX_STAMP_LEN	(5)			// read only 5 bytes (the adjusted timestamp (40:0))
#define RESP_MSG_TS_LEN			4

struct localdata{
	uint32_t partID;			// IC Part ID - read during initialisation
	uint32_t lotID;				// IC Lot ID - read during initialisation
	uint8_t bias_tune;			// bias tune code
	uint8_t dgc_otp_set;		// Flag to check if DGC values are programmed in OTP
	uint8_t vBatP;				// IC V bat read during production and stored in OTP (Vmeas @ 3V3)
	uint8_t tempP;				// IC temp read during production and stored in OTP (Tmeas @ 23C)
	uint8_t longFrames;			// Flag in non-standard long frame mode
	uint8_t otprev;				// OTP revision number (read during initialisation)
	uint8_t init_xtrim;			// initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
	uint8_t dblbuffon;			// Double RX buffer mode and DB status flag
	uint8_t channel; // new
	uint8_t extra; // new
	uint16_t sleep_mode;		// Used for automatic reloading of LDO tune and microcode at wake-up
	uint16_t ststhreshold;		// Threshold for deciding if received STS is good or bad
	dwt_spi_crc_mode_e spicrc;	// Use SPI CRC when this flag is true
	uint8_t stsconfig;			// STS configuration mode
	// uint8_t cia_diagnostic;	// CIA dignostic logging level
	// dwt_cb_data_t cbData;		// Callback data structure
	// dwt_spierrcb_t cbSPIRDErr;// Callback for SPI read error events
	dwt_cb_t cbTxDone;			// Callback for TX confirmation event
	dwt_cb_t cbRxOk;			// Callback for RX good frame event
	dwt_cb_t cbRxTo;			// Callback for RX timeout events
	dwt_cb_t cbRxErr;			// Callback for RX error events
	dwt_cb_t cbSPIErr;			// Callback for SPI error events
	dwt_cb_t cbSPIRdy;			// Callback for SPI ready events
};

struct localdata _ptr1;
struct localdata* pdw3000local=&_ptr1;

void dwt_xfer3000(uint32_t regFileID,uint16_t indx,uint16_t length,uint8_t* buffer,spi_modes_e mode){
	uint8_t header[2];			// Buffer to compose header in
	uint16_t cnt=0;				// Counter for length of a header

	uint16_t reg_file=0x1F&((regFileID+indx)>>16);
	uint16_t reg_offset=0x7F&(regFileID+indx);

	assert(reg_file<=0x1F);
	assert(reg_offset<=0x7F);
	assert(length<0x3100);
	assert(mode==DW3000_SPI_WR_BIT||mode==DW3000_SPI_RD_BIT||mode==DW3000_SPI_AND_OR_8||mode==DW3000_SPI_AND_OR_16||mode==DW3000_SPI_AND_OR_32);

	// Write message header selecting WRITE operation and addresses as appropriate
	uint16_t addr;
	addr=(reg_file<<9)|(reg_offset<<2);

	header[0]=(uint8_t)((mode|addr)>>8);		// bit7+addr[4:0]+sub_addr[6:6]
	header[1]=(uint8_t)(addr|(mode&0x03));		// EAM: subaddr[5:0]+R/W/AND_OR

	if(length==0){   // Fast Access Commands (FAC) only write operation is possible for this mode bit_7=one is W operation,bit_6=zero: FastAccess command,bit_[5..1] addr,bits_0=one: MODE of FastAccess
		assert(mode==DW3000_SPI_WR_BIT);
		header[0]=(uint8_t)((DW3000_SPI_WR_BIT>>8)|(regFileID<<1)|DW3000_SPI_FAC);
		cnt=1;
	}else
	if(reg_offset==0 && (mode==DW3000_SPI_WR_BIT||mode==DW3000_SPI_RD_BIT))	{   // Fast Access Commands with Read/Write support (FACRW) bit_7 is R/W operation,bit_6=zero: FastAccess command,bit_[5..1] addr,bits_0=zero: MODE of FastAccess
		header[0]|=DW3000_SPI_FARW;
		cnt=1;
	}else{// Extended Address Mode with Read/Write support (EAMRW) b[0]=bit_7 is R/W operation,bit_6 one=ExtendedAddressMode; b[1]=addr<<2|(mode&0x3)
		header[0]|=DW3000_SPI_EAMRW;
		cnt=2;
	}
	switch (mode){
		case DW3000_SPI_AND_OR_8:
		case DW3000_SPI_AND_OR_16:
		case DW3000_SPI_AND_OR_32:
		case DW3000_SPI_WR_BIT:{
			writetospi(cnt,header,length,buffer);
			break;
		}
		case DW3000_SPI_RD_BIT:{
			readfromspi(cnt,header,length,buffer);
			break;
		}
		default:
			while(1);
			break;
	}
}
void dwt3000_readfromdevice(uint32_t regFileID,uint16_t index,uint16_t length,uint8_t*buffer){
	dwt_xfer3000(regFileID,index,length,buffer,DW3000_SPI_RD_BIT);
}
void dwt3000_writetodevice(uint32_t regFileID,uint16_t index,uint16_t length,uint8_t*buffer){
	dwt_xfer3000(regFileID,index,length,buffer,DW3000_SPI_WR_BIT);
}

uint16_t dwt_read16bitoffsetreg(uint32_t regFileID,uint16_t regOffset) {
	uint16_t regval=0 ;
	uint8_t buffer[2] ;
	dwt3000_readfromdevice(regFileID,regOffset,2,buffer); // Read 2 bytes (16-bits) register into buffer
	regval=((uint16_t)buffer[1]<<8)+buffer[0] ;
	return regval;
}

uint8_t dwt_wrap_checkidlerc() {
	deca_sleep(2); //wait 2 ms for DW IC to get into IDLE_RC state. Poll DW IC until IDLE_RC event set. This means that DW IC is in IDLE_RC state and ready
	// uint8_t v=dwt_checkidlerc();
	uint32_t reg=((uint32_t)dwt_read16bitoffsetreg(SYS_STATUS_ID,2)<<16);
	uint8_t v=((reg&(SYS_STATUS_RCINIT_BIT_MASK))==(SYS_STATUS_RCINIT_BIT_MASK));
	return v;
}

void dwt_write16bitoffsetreg(uint32_t regFileID,uint16_t regOffset,uint16_t regval){
	uint8_t buffer[2];
	buffer[0]=(uint8_t)regval;
	buffer[1]=regval>>8 ;
	dwt3000_writetodevice(regFileID,regOffset,2,buffer);
}

uint32_t dwt_read32bitoffsetreg(uint32_t regFileID,uint16_t regOffset){
	int j;
	uint32_t regval=0;
	uint8_t buffer[4];
	dwt3000_readfromdevice(regFileID,regOffset,4,buffer); // Read 4 bytes (32-bits) register into buffer
	for(j=3;j>=0;j--){
		regval=(regval<<8)+buffer[j] ;
	}
	return (regval);
}

uint8_t dwt_read8bitoffsetreg(uint32_t regFileID,uint16_t regOffset){
	uint8_t regval;
	uint8_t buffer[1];
	dwt3000_readfromdevice(regFileID,regOffset,1,buffer);
	regval=buffer[0] ;
	return regval ;
}

void dwt_modify16bitoffsetreg(const uint32_t regFileID,const uint16_t regOffset,const uint16_t _and,const uint16_t _or){
	uint8_t buf[4];
	buf[0]=(uint8_t)_and;
	buf[1]=(uint8_t)(_and>>8);
	buf[2]=(uint8_t)_or;
	buf[3]=(uint8_t)(_or>>8);
	dwt_xfer3000(regFileID,regOffset,sizeof(buf),buf,DW3000_SPI_AND_OR_16);
}

void dwt_modify8bitoffsetreg(const uint32_t regFileID,const uint16_t regOffset,const uint8_t _and,const uint8_t _or){
	uint8_t buf[2];
	buf[0]=_and;
	buf[1]=_or;
	dwt_xfer3000(regFileID,regOffset,sizeof(buf),buf,DW3000_SPI_AND_OR_8);
}

uint32_t _dwt_otpread(uint16_t address) {
	uint32_t ret_data;
	// Set manual access mode
	dwt_write16bitoffsetreg(OTP_CFG_ID,0,0x0001);
	// set the address
	dwt_write16bitoffsetreg(OTP_ADDR_ID,0,address);
	// Assert the read strobe
	dwt_write16bitoffsetreg(OTP_CFG_ID,0,0x0002);
	// attempt a read from OTP address
	ret_data=dwt_read32bitoffsetreg(OTP_RDATA_ID,0);
	// Return the 32bit of read data
	return ret_data;
}

void _dwt_prog_ldo_and_bias_tune() {
	dwt_or16bitoffsetreg(OTP_CFG_ID,0,LDO_BIAS_KICK);
	dwt_and_or16bitoffsetreg(BIAS_CTRL_ID,0,(uint16_t)~BIAS_CTRL_BIAS_MASK,pdw3000local->bias_tune);
}
uint32_t dwt_wrap_readdevid() {
	return dwt_read32bitoffsetreg(DEV_ID_ID,0);
}
int dwt_wrap_check_dev_id() {
	uint32_t  dev_id=dwt_wrap_readdevid();
	//uprintf("DEVICE ID: 0x%08x\n",dev_id);
	if(!((DWT_C0_PDOA_DEV_ID==dev_id) || (DWT_C0_DEV_ID==dev_id))) {
		return DWT_ERROR;
	}
	return DWT_SUCCESS;
}

int dwt_wrap_initialise(uint8_t mode) {
	uint32_t ldo_tune_lo;
	uint32_t ldo_tune_hi;

	pdw3000local->dblbuffon=DBL_BUFF_OFF;		// Double buffer mode off by default / clear the flag
	pdw3000local->sleep_mode=DWT_RUNSAR;		// Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
	pdw3000local->spicrc=DWT_SPI_CRC_MODE_NO;
	pdw3000local->stsconfig=0;					// STS off
	pdw3000local->vBatP=0;
	pdw3000local->tempP=0;

	pdw3000local->cbTxDone=NULL;
	pdw3000local->cbRxOk=NULL;
	pdw3000local->cbRxTo=NULL;
	pdw3000local->cbRxErr=NULL;
	pdw3000local->cbSPIRdy=NULL;
	pdw3000local->cbSPIErr=NULL;

	// Read and validate device ID return -1 if not recognised
	if(dwt_wrap_check_dev_id()!=DWT_SUCCESS) {
		return DWT_ERROR;
	}

	// Read LDO_TUNE and BIAS_TUNE from OTP
	ldo_tune_lo=_dwt_otpread(LDOTUNELO_ADDRESS);
	ldo_tune_hi=_dwt_otpread(LDOTUNEHI_ADDRESS);
	pdw3000local->bias_tune=(_dwt_otpread(BIAS_TUNE_ADDRESS)>>16)&BIAS_CTRL_BIAS_MASK;

	if((ldo_tune_lo !=0) && (ldo_tune_hi !=0) && (pdw3000local->bias_tune !=0)) {
		_dwt_prog_ldo_and_bias_tune();
	}

	// Read DGC_CFG from OTP
	if(_dwt_otpread(DGC_TUNE_ADDRESS)==DWT_DGC_CFG0) {
		pdw3000local->dgc_otp_set=DWT_DGC_LOAD_FROM_OTP;
	}else{
		pdw3000local->dgc_otp_set=DWT_DGC_LOAD_FROM_SW;
	}

	// Load Part and Lot ID from OTP
	if(!(mode&DWT_READ_OTP_PID)) {
		pdw3000local->partID=_dwt_otpread(PARTID_ADDRESS);
	}
	if(!(mode&DWT_READ_OTP_LID)) {
		pdw3000local->lotID=_dwt_otpread(LOTID_ADDRESS);
	}
	if(!(mode&DWT_READ_OTP_BAT)) {
		pdw3000local->vBatP=(uint8_t)_dwt_otpread(VBAT_ADDRESS);
	}
	if(!(mode&DWT_READ_OTP_TMP)) {
		pdw3000local->tempP=(uint8_t)_dwt_otpread(VTEMP_ADDRESS);
	}
	if(pdw3000local->tempP==0) {		// if the reference temperature has not been programmed in OTP (early eng samples) set to default value
		pdw3000local->tempP=0x85;		// @temp of 20 deg
	}
	if(pdw3000local->vBatP==0) {		// if the reference voltage has not been programmed in OTP (early eng samples) set to default value
		pdw3000local->vBatP=0x74;		// @Vref of 3.0V
	}

	pdw3000local->otprev=(uint8_t) _dwt_otpread(OTPREV_ADDRESS);
	pdw3000local->init_xtrim=_dwt_otpread(XTRIM_ADDRESS)&0x3f;
	if(pdw3000local->init_xtrim==0) {
		pdw3000local->init_xtrim=0x2E;	// set default value
		uprintf("XTRIM OTP READ FAIL\r\n");
	}
	dwt_write8bitoffsetreg(XTAL_ID,0,pdw3000local->init_xtrim);

#define PLL_CC_ID  0x90004
#define PLL_LOCK_CODE 0x35
	uint32_t x = _dwt_otpread(0x35);
	if(x) {
		dwt_write32bitoffsetreg(PLL_CC_ID,0,x);

	}
	return DWT_SUCCESS;
}

int dwt_wrap_initialise_old(uint8_t mode) {
	uint32_t ldo_tune_lo;
	uint32_t ldo_tune_hi;

	pdw3000local->dblbuffon=DBL_BUFF_OFF;		// Double buffer mode off by default / clear the flag
	pdw3000local->sleep_mode=DWT_RUNSAR;		// Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
	pdw3000local->spicrc=DWT_SPI_CRC_MODE_NO;
	pdw3000local->stsconfig=0;					// STS off
	pdw3000local->vBatP=0;
	pdw3000local->tempP=0;

	pdw3000local->cbTxDone=NULL;
	pdw3000local->cbRxOk=NULL;
	pdw3000local->cbRxTo=NULL;
	pdw3000local->cbRxErr=NULL;
	pdw3000local->cbSPIRdy=NULL;
	pdw3000local->cbSPIErr=NULL;

	// Read and validate device ID return -1 if not recognised
	if(dwt_wrap_check_dev_id()!=DWT_SUCCESS) {
		return DWT_ERROR;
	}

	// Read LDO_TUNE and BIAS_TUNE from OTP
	ldo_tune_lo=_dwt_otpread(LDOTUNELO_ADDRESS);
	ldo_tune_hi=_dwt_otpread(LDOTUNEHI_ADDRESS);
	pdw3000local->bias_tune=(_dwt_otpread(BIAS_TUNE_ADDRESS)>>16)&BIAS_CTRL_BIAS_MASK;

	if((ldo_tune_lo !=0) && (ldo_tune_hi !=0) && (pdw3000local->bias_tune !=0)) {
		_dwt_prog_ldo_and_bias_tune();
	}

	// Read DGC_CFG from OTP
	if(_dwt_otpread(DGC_TUNE_ADDRESS)==DWT_DGC_CFG0) {
		pdw3000local->dgc_otp_set=DWT_DGC_LOAD_FROM_OTP;
	}else{
		pdw3000local->dgc_otp_set=DWT_DGC_LOAD_FROM_SW;
	}

	// Load Part and Lot ID from OTP
	if(!(mode&DWT_READ_OTP_PID)) {
		pdw3000local->partID=_dwt_otpread(PARTID_ADDRESS);
	}
	if(!(mode&DWT_READ_OTP_LID)) {
		pdw3000local->lotID=_dwt_otpread(LOTID_ADDRESS);
	}
	if(!(mode&DWT_READ_OTP_BAT)) {
		pdw3000local->vBatP=(uint8_t)_dwt_otpread(VBAT_ADDRESS);
	}
	if(!(mode&DWT_READ_OTP_TMP)) {
		pdw3000local->tempP=(uint8_t)_dwt_otpread(VTEMP_ADDRESS);
	}
	if(pdw3000local->tempP==0) {		// if the reference temperature has not been programmed in OTP (early eng samples) set to default value
		pdw3000local->tempP=0x85;		// @temp of 20 deg
	}
	if(pdw3000local->vBatP==0) {		// if the reference voltage has not been programmed in OTP (early eng samples) set to default value
		pdw3000local->vBatP=0x74;		// @Vref of 3.0V
	}

	pdw3000local->otprev=(uint8_t) _dwt_otpread(OTPREV_ADDRESS);
	pdw3000local->init_xtrim=_dwt_otpread(XTRIM_ADDRESS)&0x7f;
	if(pdw3000local->init_xtrim==0) {
		pdw3000local->init_xtrim=0x2E;	// set default value
		uprintf("XTRIM OTP READ FAIL\r\n");
	}
	dwt_write8bitoffsetreg(XTAL_ID,0,pdw3000local->init_xtrim);
	return DWT_SUCCESS ;
}

void dwt_wrap_setleds(uint8_t mode) {
	uint32_t reg;
	if(mode&DWT_LEDS_ENABLE) {
		// Set up MFIO for LED output.
		dwt_modify32bitoffsetreg(GPIO_MODE_ID,0,~(GPIO_MODE_MSGP3_MODE_BIT_MASK|GPIO_MODE_MSGP2_MODE_BIT_MASK),(GPIO_PIN2_RXLED|GPIO_PIN3_TXLED));
		// Enable LP Oscillator to run from counter and turn on de-bounce clock.
		dwt_or32bitoffsetreg(CLK_CTRL_ID,0,(CLK_CTRL_GPIO_DCLK_EN_BIT_MASK|CLK_CTRL_LP_CLK_EN_BIT_MASK));
		// Enable LEDs to blink and set default blink time.
		reg=LED_CTRL_BLINK_EN_BIT_MASK|DWT_LEDS_BLINK_TIME_DEF;
		// Make LEDs blink once if requested.
		if(mode&DWT_LEDS_INIT_BLINK) {
			reg |=LED_CTRL_FORCE_TRIGGER_BIT_MASK;
		}
		dwt_write32bitreg(LED_CTRL_ID,reg);
		// Clear force blink bits if needed.
		if(mode&DWT_LEDS_INIT_BLINK) {
			reg &=(~LED_CTRL_FORCE_TRIGGER_BIT_MASK);
			dwt_write32bitreg(LED_CTRL_ID,reg);
		}
	}else{
		// Clear the GPIO bits that are used for LED control.
		dwt_and32bitoffsetreg(GPIO_MODE_ID,0,~(GPIO_MODE_MSGP2_MODE_BIT_MASK|GPIO_MODE_MSGP3_MODE_BIT_MASK));
		dwt_and16bitoffsetreg(LED_CTRL_ID,0,(uint16_t) ~LED_CTRL_BLINK_EN_BIT_MASK);
	}

}

int dwt_wrap_rxenable(int mode) {
	uint8_t temp1 ;
	if(mode==DWT_START_RX_IMMEDIATE) {
		dwt_writefastCMD(CMD_RX);
	}else{  // delayed RX
		switch(mode&~DWT_IDLE_ON_DLY_ERR) {
			case DWT_START_RX_DELAYED:
				dwt_writefastCMD(CMD_DRX);
				break;
			case DWT_START_RX_DLY_REF:
				dwt_writefastCMD(CMD_DRX_REF);
				break;
			case DWT_START_RX_DLY_RS:
				dwt_writefastCMD(CMD_DRX_RS);
				break;
			case DWT_START_RX_DLY_TS:
				dwt_writefastCMD(CMD_DRX_TS);
				break;
			default:
				return DWT_ERROR;
		}
		temp1=dwt_read8bitoffsetreg(SYS_STATUS_ID,3);			// Read 1 byte at offset 3 to get the 4th byte out of 5
		if((temp1&(SYS_STATUS_HPDWARN_BIT_MASK>>24)) !=0){	// if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
			dwt_writefastCMD(CMD_TXRXOFF);
			if((mode&DWT_IDLE_ON_DLY_ERR)==0) {				 // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
				dwt_writefastCMD(CMD_RX);
			}
			return DWT_ERROR;
		}
	}
	return DWT_SUCCESS;
}

void dwt_wrap_setdelayedtrxtime(uint32_t starttime) {
	dwt_write32bitoffsetreg(DX_TIME_ID,0,starttime); // Note: bit 0 of this register is ignored
}

void dwt_modify32bitoffsetreg(const uint32_t regFileID,const uint16_t regOffset,const uint32_t _and,const uint32_t _or) {
	uint8_t buf[8];
	buf[0]=(uint8_t)_and;
	buf[1]=(uint8_t)(_and>>8);
	buf[2]=(uint8_t)(_and>>16);
	buf[3]=(uint8_t)(_and>>24);
	buf[4]=(uint8_t)_or;;
	buf[5]=(uint8_t)(_or>>8);
	buf[6]=(uint8_t)(_or>>16);
	buf[7]=(uint8_t)(_or>>24);
	dwt_xfer3000(regFileID,regOffset,sizeof(buf),buf,DW3000_SPI_AND_OR_32);
}

void _dwt_kick_dgc_on_wakeup(int8_t channel) {
	if(channel==5) {	// The DGC_SEL bit must be set to '0' for channel 5 and '1' for channel 9
		dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_DGC_SEL_BIT_MASK),(DWT_DGC_SEL_CH5<<OTP_CFG_DGC_SEL_BIT_OFFSET)|OTP_CFG_DGC_KICK_BIT_MASK);
	}else
	if(channel==9){
		dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_DGC_SEL_BIT_MASK),(DWT_DGC_SEL_CH9<<OTP_CFG_DGC_SEL_BIT_OFFSET)|OTP_CFG_DGC_KICK_BIT_MASK);
	}
}

const uint16_t sts_length_factors[STS_LEN_SUPPORTED]={1024,1448,2048,2896,4096,5793,8192};

void dwt_write32bitoffsetreg(uint32_t regFileID,uint16_t regOffset,uint32_t regval) {
	int j;
	uint8_t buffer[4] ;
	for(j=0;j<4;j++){
		buffer[j]=(uint8_t)regval;
		regval >>=8 ;
	}
	dwt3000_writetodevice(regFileID,regOffset,4,buffer);
}

void dwt_write8bitoffsetreg(uint32_t regFileID,uint16_t regOffset,uint8_t regval) {
	uint8_t buffer[1];
	buffer[0]=regval;
	dwt3000_writetodevice(regFileID,regOffset,1,buffer);
}

uint16_t get_sts_mnth (uint16_t cipher,uint8_t threshold,uint8_t shift_val) {
	uint32_t value;
	uint16_t mod_val;
	value=cipher*(uint32_t)threshold;
	if(shift_val==3){
		value*=SQRT_FACTOR;// Factor to sqrt(2)
		value>>=SQRT_SHIFT_VAL;
	}
	mod_val=value%MOD_VALUE+HALF_MOD;
	value>>=SHIFT_VALUE;
	if(mod_val>=MOD_VALUE){// Check if modulo greater than MOD_VALUE,if yes add 1
		value+=1;
	}
	return (uint16_t)value;
}

void dwt_wrap_setplenfine(uint8_t preambleLength) {
	dwt_write8bitoffsetreg(TX_FCTRL_HI_ID,1,preambleLength);
}

void dwt_wrap_force_clocks(int clocks) {
	if(clocks==FORCE_CLK_SYS_TX){
		uint16_t regvalue0=CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK|CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;
		// SYS_CLK_SEL=PLL
		regvalue0 |=((uint16_t) FORCE_SYSCLK_PLL)<<CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;
		// TX_CLK_SEL=ON
		regvalue0 |=((uint16_t) FORCE_CLK_PLL)<<CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;
		dwt_write16bitoffsetreg(CLK_CTRL_ID,0x0,regvalue0); // 0x110004,0,0x1822
	}
	if(clocks==FORCE_CLK_AUTO) {
		// Restore auto clock mode
		dwt_write16bitoffsetreg(CLK_CTRL_ID,0x0,(uint16_t) DWT_AUTO_CLKS);  // we only need to restore the low 16 bits as they are the only ones to change as a result of  FORCE_CLK_SYS_TX
	}
}

void dwt_wrap_setdwstate_old(uint8_t state) {
	if(state==DWT_DW_IDLE) {// Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
		// PLL should be configured prior to this,and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)
		// switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4,need to switch to auto prior to setting auto INIT2IDLE bit
		dwt_wrap_force_clocks(FORCE_CLK_AUTO);
		dwt_or8bitoffsetreg(SEQ_CTRL_ID,0x01,SEQ_CTRL_AINIT2IDLE_BIT_MASK>>8); // 0x110008, 1, 1
	}else
	if(state==DWT_DW_IDLE_RC) {  // Change state to IDLE_RC and clear auto INIT2IDLE bit
		// switch clock to FOSC
		dwt_or8bitoffsetreg(CLK_CTRL_ID,0,FORCE_SYSCLK_FOSC);
		// clear the auto INIT2IDLE bit and set FORCE2INIT
		dwt_modify32bitoffsetreg(SEQ_CTRL_ID,0x0,(uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK,SEQ_CTRL_FORCE2INIT_BIT_MASK);
		// clear force bits (device will stay in IDLE_RC)
		dwt_and8bitoffsetreg(SEQ_CTRL_ID,0x2,(uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
		// switch clock to auto
		dwt_wrap_force_clocks(FORCE_CLK_AUTO);
	}else{ // The SPI rate needs to be <=7MHz as device is switching to INIT_RC state
		dwt_or8bitoffsetreg(CLK_CTRL_ID,0,FORCE_SYSCLK_FOSCDIV4);
		// clear the auto INIT2IDLE bit and set FORCE2INIT
		dwt_modify32bitoffsetreg(SEQ_CTRL_ID,0x0,(uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK,SEQ_CTRL_FORCE2INIT_BIT_MASK);
		dwt_and8bitoffsetreg(SEQ_CTRL_ID,0x2,(uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
	}
}

void dwt_wrap_setdwstate(uint8_t state) {
	if(state==DWT_DW_IDLE) {// Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
		// PLL should be configured prior to this,and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)
		// switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4,need to switch to auto prior to setting auto INIT2IDLE bit
		dwt_wrap_force_clocks(FORCE_CLK_AUTO);
		dwt_modify32bitoffsetreg(PLL_CAL_ID, 0, 0xffffffff, 0x102); //new:  0x90008, 0, 0xffffffff, 0x102
		dwt_modify8bitoffsetreg(SEQ_CTRL_ID, 1, 0xff, 1); //new: 0x110008, 1, 0xff, 1
	}else
	if(state==DWT_DW_IDLE_RC) {  // Change state to IDLE_RC and clear auto INIT2IDLE bit
		// switch clock to FOSC
		dwt_or8bitoffsetreg(CLK_CTRL_ID,0,FORCE_SYSCLK_FOSC);
		// clear the auto INIT2IDLE bit and set FORCE2INIT
		dwt_modify32bitoffsetreg(SEQ_CTRL_ID,0x0,(uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK,SEQ_CTRL_FORCE2INIT_BIT_MASK);
		// clear force bits (device will stay in IDLE_RC)
		dwt_and8bitoffsetreg(SEQ_CTRL_ID,0x2,(uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
		// switch clock to auto
		dwt_wrap_force_clocks(FORCE_CLK_AUTO);
	}else{ // The SPI rate needs to be <=7MHz as device is switching to INIT_RC state
		dwt_or8bitoffsetreg(CLK_CTRL_ID,0,FORCE_SYSCLK_FOSCDIV4);
		// clear the auto INIT2IDLE bit and set FORCE2INIT
		dwt_modify32bitoffsetreg(SEQ_CTRL_ID,0x0,(uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK,SEQ_CTRL_FORCE2INIT_BIT_MASK);
		dwt_and8bitoffsetreg(SEQ_CTRL_ID,0x2,(uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
	}
}

void dwt_wrap_configmrxlut(uint8_t channel) {
	uint32_t lut0,lut1,lut2,lut3,lut4,lut5,lut6=0;
	if(channel==5) {
		lut0=(uint32_t)CH5_DGC_LUT_0;
		lut1=(uint32_t)CH5_DGC_LUT_1;
		lut2=(uint32_t)CH5_DGC_LUT_2;
		lut3=(uint32_t)CH5_DGC_LUT_3;
		lut4=(uint32_t)CH5_DGC_LUT_4;
		lut5=(uint32_t)CH5_DGC_LUT_5;
		lut6=(uint32_t)CH5_DGC_LUT_6;
	}else{
		lut0=(uint32_t)CH9_DGC_LUT_0;
		lut1=(uint32_t)CH9_DGC_LUT_1;
		lut2=(uint32_t)CH9_DGC_LUT_2;
		lut3=(uint32_t)CH9_DGC_LUT_3;
		lut4=(uint32_t)CH9_DGC_LUT_4;
		lut5=(uint32_t)CH9_DGC_LUT_5;
		lut6=(uint32_t)CH9_DGC_LUT_6;
	}
	dwt_write32bitoffsetreg(DGC_LUT_0_CFG_ID,0x0,lut0);
	dwt_write32bitoffsetreg(DGC_LUT_1_CFG_ID,0x0,lut1);
	dwt_write32bitoffsetreg(DGC_LUT_2_CFG_ID,0x0,lut2);
	dwt_write32bitoffsetreg(DGC_LUT_3_CFG_ID,0x0,lut3);
	dwt_write32bitoffsetreg(DGC_LUT_4_CFG_ID,0x0,lut4);
	dwt_write32bitoffsetreg(DGC_LUT_5_CFG_ID,0x0,lut5);
	dwt_write32bitoffsetreg(DGC_LUT_6_CFG_ID,0x0,lut6);
	dwt_write32bitoffsetreg(DGC_CFG0_ID,0x0,DWT_DGC_CFG0);
	dwt_write32bitoffsetreg(DGC_CFG1_ID,0x0,DWT_DGC_CFG1);
}

#define ERR_RX_CAL_FAIL					0x1fffffff

int dwt_wrap_run_pgfcal() {
	int result=DWT_SUCCESS;
	uint32_t data;
	uint32_t val=0;
	uint8_t cnt,flag;
	data=(((uint32_t)0x02)<<RX_CAL_CFG_COMP_DLY_BIT_OFFSET)|(RX_CAL_CFG_CAL_MODE_BIT_MASK&0x1);			// put into cal mode. Turn on delay mode
	dwt_write32bitoffsetreg(RX_CAL_CFG_ID,0x0,data);
	dwt_or8bitoffsetreg(RX_CAL_CFG_ID,0x0,RX_CAL_CFG_CAL_EN_BIT_MASK);									// Trigger PGF Cal
	for (flag=1,cnt=0;cnt<MAX_RETRIES_FOR_PGF;cnt++) {
		deca_usleep(DELAY_20uUSec);
		if(dwt_read8bitoffsetreg(RX_CAL_STS_ID,0x0)==1) {												// PGF cal is complete
			//uprintf("PGF cal complete..\n");
			flag=0;
			break;
		}
	}
	if(flag) {
		uprintf("PGF CAL ERROR\n");
		result=DWT_ERROR;
	}
	// Put into normal mode
	dwt_write8bitoffsetreg(RX_CAL_CFG_ID,0x0,0);
	dwt_write8bitoffsetreg(RX_CAL_STS_ID,0x0,1); // clear the status
	dwt_or8bitoffsetreg(RX_CAL_CFG_ID,0x2,0x1); // enable reading
	val=dwt_read32bitoffsetreg(RX_CAL_RESI_ID,0x0);
	if(val==ERR_RX_CAL_FAIL) {	// PGF I Cal Fail
		uprintf("PGF I CAL ERROR\n");
		result=DWT_ERROR;
	}
	val=dwt_read32bitoffsetreg(RX_CAL_RESQ_ID,0x0);
	if(val==ERR_RX_CAL_FAIL) {	// PGF Q Cal Fail
		uprintf("PGF Q CAL ERROR\n");
		result=DWT_ERROR;
	}
	return result;
}

int dwt_wrap_pgf_cal(uint8_t ldoen) {
	uint8_t temp;
	uint16_t val;
	if(ldoen==1) {// PGF needs LDOs turned on - ensure PGF LDOs are enabled
		val=dwt_read16bitoffsetreg(LDO_CTRL_ID,0);
		dwt_or16bitoffsetreg(LDO_CTRL_ID,0,(LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK |	LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK|LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
	}// Run PGF Cal
	temp=dwt_wrap_run_pgfcal();
	if(ldoen==1) {	// Turn off RX LDOs if previously off
		dwt_and16bitoffsetreg(LDO_CTRL_ID,0,val); // restore LDO values
	}
	return temp;
}

int dwt_wrap_configure(dwt_config_t*config){
	pdw3000local->dgc_otp_set=DWT_DGC_LOAD_FROM_SW;
	pdw3000local->longFrames=0;			// Flag in non-standard long frame mode
	pdw3000local->sleep_mode=0;			// Used for automatic reloading of LDO tune and microcode at wake-up
	pdw3000local->ststhreshold=0;		// Threshold for deciding if received STS is good or bad
	pdw3000local->stsconfig=0;			// STS configuration mode

	uint8_t chan=config->chan,cnt,flag;
	uint32_t temp;
	uint8_t scp=((config->rxCode>24)||(config->txCode>24)) ? 1:0;
	uint8_t mode=(config->phrMode==DWT_PHRMODE_EXT) ? SYS_CFG_PHR_MODE_BIT_MASK:0;
	uint16_t sts_len;
	int error=DWT_SUCCESS;
#ifdef DWT_API_ERROR_CHECK
	assert((config->dataRate==DWT_BR_6M8)||(config->dataRate==DWT_BR_850K));
	assert(config->rxPAC <=DWT_PAC4);
	assert((chan==5)||(chan==9));
	assert((config->txPreambLength==DWT_PLEN_32)||(config->txPreambLength==DWT_PLEN_64)||(config->txPreambLength==DWT_PLEN_72)||(config->txPreambLength==DWT_PLEN_128)||(config->txPreambLength==DWT_PLEN_256)
			||(config->txPreambLength==DWT_PLEN_512)||(config->txPreambLength==DWT_PLEN_1024)||(config->txPreambLength==DWT_PLEN_1536)
			||(config->txPreambLength==DWT_PLEN_2048)||(config->txPreambLength==DWT_PLEN_4096));
	assert((config->phrMode==DWT_PHRMODE_STD)||(config->phrMode==DWT_PHRMODE_EXT));
	assert((config->phrRate==DWT_PHRRATE_STD)||(config->phrRate==DWT_PHRRATE_DTA));
	assert((config->pdoaMode==DWT_PDOA_M0)||(config->pdoaMode==DWT_PDOA_M1)||(config->pdoaMode==DWT_PDOA_M3));
	assert(((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_OFF)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_1)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_2)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_ND)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_SDC)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==(DWT_STS_MODE_1|DWT_STS_MODE_SDC))
		||((config->stsMode&DWT_STS_CONFIG_MASK)==(DWT_STS_MODE_2|DWT_STS_MODE_SDC))
		||((config->stsMode&DWT_STS_CONFIG_MASK)==(DWT_STS_MODE_ND|DWT_STS_MODE_SDC))
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_CONFIG_MASK));
#endif
	int preamble_len;
	switch (config->txPreambLength){
		case DWT_PLEN_32:
			preamble_len=32;
			break;
		case DWT_PLEN_64:
			preamble_len=64;
			break;
		case DWT_PLEN_72:
			preamble_len=72;
			break;
		case DWT_PLEN_128:
			preamble_len=128;
			break;
		default:
			preamble_len=256;
			break;
	}
	pdw3000local->sleep_mode &=(~(DWT_ALT_OPS|DWT_SEL_OPS3));  // clear the sleep mode ALT_OPS bit
	// ~(0xc0|0x20) = 0xff1f, ok
	pdw3000local->longFrames=config->phrMode ;
	sts_len=GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength));
	pdw3000local->ststhreshold=(uint16_t)(sts_len * 0x26668 >>15); // new
	pdw3000local->stsconfig=config->stsMode;
	// SYS_CFG
	// clear the PHR Mode,PHR Rate,STS Protocol,SDC,PDOA Mode,then set the relevant bits according to configuration of the PHR Mode,PHR Rate,STS Protocol,SDC,PDOA Mode,
	dwt_modify32bitoffsetreg(SYS_CFG_ID,0,
			(uint32_t) ~(SYS_CFG_PHR_MODE_BIT_MASK|SYS_CFG_PHR_6M8_BIT_MASK|SYS_CFG_CP_SPC_BIT_MASK|SYS_CFG_PDOA_MODE_BIT_MASK|SYS_CFG_CP_SDC_BIT_MASK),
			((uint32_t)config->pdoaMode)<<SYS_CFG_PDOA_MODE_BIT_OFFSET | // ok:  pdoaMode < 0x10
			((uint16_t)config->stsMode&DWT_STS_CONFIG_MASK)<<SYS_CFG_CP_SPC_BIT_OFFSET | // ok: (stsMode & 0xb) << 0xc
			(SYS_CFG_PHR_6M8_BIT_MASK&((uint32_t)config->phrRate<<SYS_CFG_PHR_6M8_BIT_OFFSET)) | // ok: same as (phrRate & 0x1) << 5
			mode);
	//dwt_modify32bitoffsetreg(SYS_CFG_ID,0,~(SYS_CFG_PHR_MODE_BIT_MASK|SYS_CFG_PHR_6M8_BIT_MASK|SYS_CFG_CP_SPC_BIT_MASK|SYS_CFG_PDOA_MODE_BIT_MASK|SYS_CFG_CP_SDC_BIT_MASK),((uint32_t)config->pdoaMode)<<SYS_CFG_PDOA_MODE_BIT_OFFSET|((uint16_t)config->stsMode&DWT_STS_CONFIG_MASK)<<SYS_CFG_CP_SPC_BIT_OFFSET|(SYS_CFG_PHR_6M8_BIT_MASK&((uint32_t)config->phrRate<<SYS_CFG_PHR_6M8_BIT_OFFSET))|mode);

	if(scp){
FATAL("path not taken");
		// configure OPS tables for SCP mode
		pdw3000local->sleep_mode|=DWT_ALT_OPS|DWT_SEL_OPS1;  // configure correct OPS table is kicked on wakeup
		dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_OPS_ID_BIT_MASK),DWT_OPSET_SCP|OTP_CFG_OPS_KICK_BIT_MASK);

		dwt_write32bitoffsetreg(IP_CONFIG_LO_ID,0,IP_CONFIG_LO_SCP);				// Set this if Ipatov analysis is used in SCP mode
		dwt_write32bitoffsetreg(IP_CONFIG_HI_ID,0,IP_CONFIG_HI_SCP);

		dwt_write32bitoffsetreg(STS_CONFIG_LO_ID,0,STS_CONFIG_LO_SCP);
		dwt_write8bitoffsetreg(STS_CONFIG_HI_ID,0,STS_CONFIG_HI_SCP);
	}else{
		uint16_t sts_mnth;
		if(config->stsMode !=DWT_STS_MODE_OFF){
FATAL("path not taken");
			// configure CIA STS lower bound
			if((config->pdoaMode==DWT_PDOA_M1)||(config->pdoaMode==DWT_PDOA_M0)){
				// In PDOA mode 1,number of accumulated symbols is the whole length of the STS
				sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)],CIA_MANUALLOWERBOUND_TH_64,3);
			}else{
				// In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
				sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)],CIA_MANUALLOWERBOUND_TH_64,4);
			}
			preamble_len+=(sts_len)*8;
			dwt_modify16bitoffsetreg(STS_CONFIG_LO_ID,2,(uint16_t)~(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK>>16),sts_mnth&0x7F);
		}
		if(preamble_len>=256){// configure OPS tables for non-SCP mode
FATAL("path not taken");
			pdw3000local->sleep_mode|=DWT_ALT_OPS|DWT_SEL_OPS0;  // ok: sleep_mode = sleep_mode|0x20
			dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_OPS_ID_BIT_MASK),DWT_OPSET_LONG|OTP_CFG_OPS_KICK_BIT_MASK); // ok:  ~(0x1800) = 0xe78ff ,  0x400
		}else{
			// old: dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_OPS_ID_BIT_MASK),DWT_OPSET_SHORT|OTP_CFG_OPS_KICK_BIT_MASK);
			// new:
			dwt_modify16bitoffsetreg(OTP_CFG_ID, 0x0, 0xe7ff, 0x1400);
			// raw spi:
			//uint8_t H1[] = {0xdc, 0x5b};
			//uint8_t B1[] = {0x00,0xff,0xff,0xbf,0x94,0x00,0x00,0x00};
			//writetospi(sizeof(H1),H1,sizeof(B1),B1); // JH
		}
	}
//	dwt_modify8bitoffsetreg(DTUNE0_ID,0,(uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK,config->rxPAC);
	if(config->pdoaMode == 1) {
FATAL("path not taken");
		dwt_modify8bitoffsetreg(DTUNE0_ID,0,0xec,config->rxPAC);
	} else
	{
		dwt_modify8bitoffsetreg(DTUNE0_ID,0,0xfc,config->rxPAC | 0x10);
	}
	//
	//dwt_modify8bitoffsetreg(DTUNE0_ID,0,(uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK,0x10); // JH
	dwt_write8bitoffsetreg(STS_CFG0_ID,0,sts_len-1);							// Starts from 0 that is why -1

	if(config->txPreambLength==DWT_PLEN_72){
FATAL("path not taken");
		//dwt_wrap_setplenfine(8); // value 8 sets fine preamble length to 72 symbols - this is needed to set 72 length.
		dwt_write8bitoffsetreg(TX_FCTRL_HI_ID,1,8);
	}else{
		dwt_write8bitoffsetreg(TX_FCTRL_HI_ID,1,0);
		//dwt_wrap_setplenfine(0); // clear the setting in the FINE_PLEN register.
	}

	dwt_write32bitoffsetreg(DTUNE3_ID,0,PD_THRESH_NO_DATA); // JH

	// CHAN_CTRL
	temp=dwt_read32bitoffsetreg(CHAN_CTRL_ID,0);
	temp&=(~(CHAN_CTRL_RX_PCODE_BIT_MASK|CHAN_CTRL_TX_PCODE_BIT_MASK|CHAN_CTRL_SFD_TYPE_BIT_MASK|CHAN_CTRL_RF_CHAN_BIT_MASK)); // ok: ~(0x1f00|0xf8|0x6|0x1) = 0xffffe000
	if(chan==9) temp|=CHAN_CTRL_RF_CHAN_BIT_MASK;

	temp|=(CHAN_CTRL_RX_PCODE_BIT_MASK&((uint32_t)config->rxCode<<CHAN_CTRL_RX_PCODE_BIT_OFFSET));  // 0x1f00 & (rxCode  << 8) = (0x1f & rxCode) << 8
	temp|=(CHAN_CTRL_TX_PCODE_BIT_MASK&((uint32_t)config->txCode<<CHAN_CTRL_TX_PCODE_BIT_OFFSET));  //   0xf8 & (txCode  << 3) = (0x1f & txCode) << 3
	temp|=(CHAN_CTRL_SFD_TYPE_BIT_MASK&((uint32_t)config->sfdType<<CHAN_CTRL_SFD_TYPE_BIT_OFFSET)); //    0x6 & (sfdType << 1) = (0x3 & sfdType) << 1  , ok

	dwt_write32bitoffsetreg(CHAN_CTRL_ID,0,temp);

	// TX_FCTRL
	// Set up TX Preamble Size,PRF and Data Rate
	dwt_modify32bitoffsetreg(TX_FCTRL_ID,0,~(TX_FCTRL_TXBR_BIT_MASK|TX_FCTRL_TXPSR_BIT_MASK), // ~0x0000f400 =  FFFF0BFF
			((uint32_t)config->dataRate<<TX_FCTRL_TXBR_BIT_OFFSET)|((uint32_t) config->txPreambLength)<<TX_FCTRL_TXPSR_BIT_OFFSET); // ok (dataRate<<10)|(txPreambLength<<12)

	// DTUNE (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if(config->sfdTO==0){
FATAL("path not taken");
		config->sfdTO=DWT_SFDTOC_DEF;
	}
	dwt_write16bitoffsetreg(DTUNE0_ID,2,config->sfdTO);

	uint8_t xx = dwt_read8bitoffsetreg(0xf0030, 2); // SYS_STATE_LO_ID ??
	if(pdw3000local->channel == chan) {
FATAL("path not taken");
		if(xx==3) goto LABEL_00014746;
	}
	else if(xx == 3) {
FATAL("path not taken");
		dwt_wrap_setdwstate(DWT_DW_IDLE_RC); // ull_setdwstate(2)
	}

	// RF
	if(chan==9){
FATAL("path not taken");
		// Setup TX analog for ch9
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID,0,RF_TXCTRL_CH9);
		dwt_write16bitoffsetreg(PLL_CFG_ID,0,RF_PLL_CFG_CH9);
		// Setup RX analog for ch9
		dwt_write32bitoffsetreg(RX_CTRL_HI_ID,0,RF_RXCTRL_CH9);
	}else{
		// Setup TX analog for ch5
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID,0,RF_TXCTRL_CH5); // ok
		dwt_write16bitoffsetreg(PLL_CFG_ID,0,RF_PLL_CFG_CH5); // ok
	}

	dwt_write8bitoffsetreg(LDO_RLOAD_ID,1,LDO_RLOAD_VAL_B1); // ok 0x70050, 1, 0x14
	dwt_write8bitoffsetreg(TX_CTRL_LO_ID,2,RF_TXCTRL_LO_B2); // ok 0x70018, 2, 0x0e
//        printf("done?\n");
	dwt_write8bitoffsetreg(PLL_CAL_ID,0,RF_PLL_CFG_LD);		// ok	0x90008, 0, 0x81 // Extend the lock delay

	// Verify PLL lock bit is cleared
	//dwt_write32bitoffsetreg(SYS_STATUS_ID,0,SYS_STATUS_CP_LOCK_BIT_MASK); // was write32 now write8
	dwt_write8bitoffsetreg(SYS_STATUS_ID,0,SYS_STATUS_CP_LOCK_BIT_MASK); //0x44,0,0x2

	// auto cal the PLL and change to IDLE_PLL state
	dwt_wrap_setdwstate(DWT_DW_IDLE); // ok calls ull_setdwstate

	for (flag=1,cnt=0;cnt<MAX_RETRIES_FOR_PLL;cnt++){
		deca_usleep(DELAY_20uUSec); // ok: 20
		if((dwt_read8bitoffsetreg(SYS_STATUS_ID,0)&SYS_STATUS_CP_LOCK_BIT_MASK)){// PLL is locked  // ok
			UART_puts("PLL is locked..");
			UART_puts("\r\n");
			flag=0;
			break;
		}
	}
	if(flag){
		UART_puts("PLL LOCKING ERROR");
		UART_puts("\r\n");
		return DWT_ERR_PLL_LOCK; // was DWT_ERROR
	}

	pdw3000local->channel = chan; // new

LABEL_00014746:
	if((config->rxCode >=9) && (config->rxCode <=24)){ // only enable DGC for PRF 64
		// load RX LUTs If the OTP has DGC info programmed into it,do a manual kick from OTP.
		if(pdw3000local->dgc_otp_set==DWT_DGC_LOAD_FROM_OTP){
			_dwt_kick_dgc_on_wakeup(chan);
		}else{// Else we manually program hard-coded values into the DGC registers.
			dwt_wrap_configmrxlut(chan);
		}
		dwt_modify16bitoffsetreg(DGC_CFG_ID,0x0,0x81ff, 0x6400); // ok: 0x30018, 0, ...
	}else{
FATAL("path not taken");
		dwt_and8bitoffsetreg(DGC_CFG_ID,0x0,(uint8_t)~DGC_CFG_RX_TUNE_EN_BIT_MASK); // 0x30018, 0, 0xfe
	}

#define DTUNE4_ID 0x60010
	if(preamble_len <= 64) {
FATAL("path not taken");
		dwt_modify32bitoffsetreg(DTUNE4_ID, 0x0, 0xffffff, 0x14000000);
	} else {
		dwt_modify32bitoffsetreg(DTUNE4_ID, 0x0, 0xffffff, 0x20000000);
	}

	// PGF
	error=dwt_wrap_pgf_cal(1);  // if the RX calibration routine fails the device receiver performance will be severely affected,the application should reset and try again
	return error;
}



int dwt_wrap_configure_old(dwt_config_t*config){
	pdw3000local->dgc_otp_set=DWT_DGC_LOAD_FROM_SW;
	pdw3000local->longFrames=0;			// Flag in non-standard long frame mode
	pdw3000local->sleep_mode=0;			// Used for automatic reloading of LDO tune and microcode at wake-up
	pdw3000local->ststhreshold=0;		// Threshold for deciding if received STS is good or bad
	pdw3000local->stsconfig=0;			// STS configuration mode

	uint8_t chan=config->chan,cnt,flag;
	uint32_t temp;
	uint8_t scp=((config->rxCode>24)||(config->txCode>24)) ? 1:0;
	uint8_t mode=(config->phrMode==DWT_PHRMODE_EXT) ? SYS_CFG_PHR_MODE_BIT_MASK:0;
	uint16_t sts_len;
	int error=DWT_SUCCESS;
#ifdef DWT_API_ERROR_CHECK
	assert((config->dataRate==DWT_BR_6M8)||(config->dataRate==DWT_BR_850K));
	assert(config->rxPAC <=DWT_PAC4);
	assert((chan==5)||(chan==9));
	assert((config->txPreambLength==DWT_PLEN_32)||(config->txPreambLength==DWT_PLEN_64)||(config->txPreambLength==DWT_PLEN_72)||(config->txPreambLength==DWT_PLEN_128)||(config->txPreambLength==DWT_PLEN_256)
			||(config->txPreambLength==DWT_PLEN_512)||(config->txPreambLength==DWT_PLEN_1024)||(config->txPreambLength==DWT_PLEN_1536)
			||(config->txPreambLength==DWT_PLEN_2048)||(config->txPreambLength==DWT_PLEN_4096));
	assert((config->phrMode==DWT_PHRMODE_STD)||(config->phrMode==DWT_PHRMODE_EXT));
	assert((config->phrRate==DWT_PHRRATE_STD)||(config->phrRate==DWT_PHRRATE_DTA));
	assert((config->pdoaMode==DWT_PDOA_M0)||(config->pdoaMode==DWT_PDOA_M1)||(config->pdoaMode==DWT_PDOA_M3));
	assert(((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_OFF)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_1)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_2)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_ND)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_MODE_SDC)
		||((config->stsMode&DWT_STS_CONFIG_MASK)==(DWT_STS_MODE_1|DWT_STS_MODE_SDC))
		||((config->stsMode&DWT_STS_CONFIG_MASK)==(DWT_STS_MODE_2|DWT_STS_MODE_SDC))
		||((config->stsMode&DWT_STS_CONFIG_MASK)==(DWT_STS_MODE_ND|DWT_STS_MODE_SDC))
		||((config->stsMode&DWT_STS_CONFIG_MASK)==DWT_STS_CONFIG_MASK));
#endif
	int preamble_len;
	switch (config->txPreambLength){
		case DWT_PLEN_32:
			preamble_len=32;
			break;
		case DWT_PLEN_64:
			preamble_len=64;
			break;
		case DWT_PLEN_72:
			preamble_len=72;
			break;
		case DWT_PLEN_128:
			preamble_len=128;
			break;
		default:
			preamble_len=256;
			break;
	}
	pdw3000local->sleep_mode &=(~(DWT_ALT_OPS|DWT_SEL_OPS3));  // clear the sleep mode ALT_OPS bit
	pdw3000local->longFrames=config->phrMode ;
	sts_len=GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength));
	pdw3000local->ststhreshold=(int16_t)((((uint32_t)sts_len)*8)*STSQUAL_THRESH_64);
	pdw3000local->stsconfig=config->stsMode;
	// SYS_CFG
	// clear the PHR Mode,PHR Rate,STS Protocol,SDC,PDOA Mode,then set the relevant bits according to configuration of the PHR Mode,PHR Rate,STS Protocol,SDC,PDOA Mode,
	dwt_modify32bitoffsetreg(SYS_CFG_ID,0,(uint32_t)~(SYS_CFG_PHR_MODE_BIT_MASK|SYS_CFG_PHR_6M8_BIT_MASK|SYS_CFG_CP_SPC_BIT_MASK|SYS_CFG_PDOA_MODE_BIT_MASK|SYS_CFG_CP_SDC_BIT_MASK),((uint32_t)config->pdoaMode)<<SYS_CFG_PDOA_MODE_BIT_OFFSET|((uint16_t)config->stsMode&DWT_STS_CONFIG_MASK)<<SYS_CFG_CP_SPC_BIT_OFFSET|(SYS_CFG_PHR_6M8_BIT_MASK&((uint32_t)config->phrRate<<SYS_CFG_PHR_6M8_BIT_OFFSET))|mode);
	if(scp){
		// configure OPS tables for SCP mode
		pdw3000local->sleep_mode|=DWT_ALT_OPS|DWT_SEL_OPS1;  // configure correct OPS table is kicked on wakeup
		dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_OPS_ID_BIT_MASK),DWT_OPSET_SCP|OTP_CFG_OPS_KICK_BIT_MASK);

		dwt_write32bitoffsetreg(IP_CONFIG_LO_ID,0,IP_CONFIG_LO_SCP);				// Set this if Ipatov analysis is used in SCP mode
		dwt_write32bitoffsetreg(IP_CONFIG_HI_ID,0,IP_CONFIG_HI_SCP);

		dwt_write32bitoffsetreg(STS_CONFIG_LO_ID,0,STS_CONFIG_LO_SCP);
		dwt_write8bitoffsetreg(STS_CONFIG_HI_ID,0,STS_CONFIG_HI_SCP);
	}else{
		uint16_t sts_mnth;
		if(config->stsMode !=DWT_STS_MODE_OFF){
			// configure CIA STS lower bound
			if((config->pdoaMode==DWT_PDOA_M1)||(config->pdoaMode==DWT_PDOA_M0)){
				// In PDOA mode 1,number of accumulated symbols is the whole length of the STS
				sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)],CIA_MANUALLOWERBOUND_TH_64,3);
			}else{
				// In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
				sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)],CIA_MANUALLOWERBOUND_TH_64,4);
			}
			preamble_len+=(sts_len)*8;
			dwt_modify16bitoffsetreg(STS_CONFIG_LO_ID,2,(uint16_t)~(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK>>16),sts_mnth&0x7F);
		}
		if(preamble_len>=256){// configure OPS tables for non-SCP mode
			pdw3000local->sleep_mode|=DWT_ALT_OPS|DWT_SEL_OPS0;
			dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_OPS_ID_BIT_MASK),DWT_OPSET_LONG|OTP_CFG_OPS_KICK_BIT_MASK);
		}else{
			dwt_modify32bitoffsetreg(OTP_CFG_ID,0,~(OTP_CFG_OPS_ID_BIT_MASK),DWT_OPSET_SHORT|OTP_CFG_OPS_KICK_BIT_MASK);
		}

	}
	dwt_modify8bitoffsetreg(DTUNE0_ID,0,(uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK,config->rxPAC);
	dwt_write8bitoffsetreg(STS_CFG0_ID,0,sts_len-1);							// Starts from 0 that is why -1
	if(config->txPreambLength==DWT_PLEN_72){
		dwt_wrap_setplenfine(8); // value 8 sets fine preamble length to 72 symbols - this is needed to set 72 length.
	}else{
		dwt_wrap_setplenfine(0); // clear the setting in the FINE_PLEN register.
	}
	if((config->stsMode&DWT_STS_MODE_ND)==DWT_STS_MODE_ND){
		// configure lower preamble detection threshold for no data STS mode
		dwt_write32bitoffsetreg(DTUNE3_ID,0,PD_THRESH_NO_DATA);
	}else{
		// configure default preamble detection threshold for other modes
		dwt_write32bitoffsetreg(DTUNE3_ID,0,PD_THRESH_DEFAULT);
	}
	// CHAN_CTRL
	temp=dwt_read32bitoffsetreg(CHAN_CTRL_ID,0);
	temp&=(~(CHAN_CTRL_RX_PCODE_BIT_MASK|CHAN_CTRL_TX_PCODE_BIT_MASK|CHAN_CTRL_SFD_TYPE_BIT_MASK|CHAN_CTRL_RF_CHAN_BIT_MASK));
	if(chan==9) temp|=CHAN_CTRL_RF_CHAN_BIT_MASK;

	temp|=(CHAN_CTRL_RX_PCODE_BIT_MASK&((uint32_t)config->rxCode<<CHAN_CTRL_RX_PCODE_BIT_OFFSET));
	temp|=(CHAN_CTRL_TX_PCODE_BIT_MASK&((uint32_t)config->txCode<<CHAN_CTRL_TX_PCODE_BIT_OFFSET));
	temp|=(CHAN_CTRL_SFD_TYPE_BIT_MASK&((uint32_t)config->sfdType<<CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

	dwt_write32bitoffsetreg(CHAN_CTRL_ID,0,temp);

	// TX_FCTRL
	// Set up TX Preamble Size,PRF and Data Rate
	dwt_modify32bitoffsetreg(TX_FCTRL_ID,0,~(TX_FCTRL_TXBR_BIT_MASK|TX_FCTRL_TXPSR_BIT_MASK),((uint32_t)config->dataRate<<TX_FCTRL_TXBR_BIT_OFFSET)|((uint32_t) config->txPreambLength)<<TX_FCTRL_TXPSR_BIT_OFFSET);

	// DTUNE (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if(config->sfdTO==0){
		config->sfdTO=DWT_SFDTOC_DEF;
	}
	dwt_write16bitoffsetreg(DTUNE0_ID,2,config->sfdTO);
	// RF
	if(chan==9){
		// Setup TX analog for ch9
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID,0,RF_TXCTRL_CH9);
		dwt_write16bitoffsetreg(PLL_CFG_ID,0,RF_PLL_CFG_CH9);
		// Setup RX analog for ch9
		dwt_write32bitoffsetreg(RX_CTRL_HI_ID,0,RF_RXCTRL_CH9);
	}else{
		// Setup TX analog for ch5
		dwt_write32bitoffsetreg(TX_CTRL_HI_ID,0,RF_TXCTRL_CH5);
		dwt_write16bitoffsetreg(PLL_CFG_ID,0,RF_PLL_CFG_CH5);
	}

	dwt_write8bitoffsetreg(LDO_RLOAD_ID,1,LDO_RLOAD_VAL_B1);
	dwt_write8bitoffsetreg(TX_CTRL_LO_ID,2,RF_TXCTRL_LO_B2);

	dwt_write8bitoffsetreg(PLL_CAL_ID,0,RF_PLL_CFG_LD);					// Extend the lock delay

	// Verify PLL lock bit is cleared
	dwt_write32bitoffsetreg(SYS_STATUS_ID,0,SYS_STATUS_CP_LOCK_BIT_MASK);

	// auto cal the PLL and change to IDLE_PLL state
	dwt_wrap_setdwstate(DWT_DW_IDLE);

	for (flag=1,cnt=0;cnt<MAX_RETRIES_FOR_PLL;cnt++){
		deca_usleep(DELAY_20uUSec);
		if((dwt_read8bitoffsetreg(SYS_STATUS_ID,0)&SYS_STATUS_CP_LOCK_BIT_MASK)){// PLL is locked
			//uprintf("PLL is locked..\n");
			flag=0;
			break;
		}
	}
	if(flag){
		uprintf("PLL LOCKING ERROR\n");
		return  DWT_ERROR;
	}
	if((config->rxCode >=9) && (config->rxCode <=24)){ // only enable DGC for PRF 64
		// load RX LUTs If the OTP has DGC info programmed into it,do a manual kick from OTP.
		if(pdw3000local->dgc_otp_set==DWT_DGC_LOAD_FROM_OTP){
			_dwt_kick_dgc_on_wakeup(chan);
		}else{// Else we manually program hard-coded values into the DGC registers.
			dwt_wrap_configmrxlut(chan);
		}
		dwt_modify16bitoffsetreg(DGC_CFG_ID,0x0,(uint16_t)~DGC_CFG_THR_64_BIT_MASK,DWT_DGC_CFG<<DGC_CFG_THR_64_BIT_OFFSET);
	}else{
		dwt_and8bitoffsetreg(DGC_CFG_ID,0x0,(uint8_t)~DGC_CFG_RX_TUNE_EN_BIT_MASK);
	}
	// PGF
	error=dwt_wrap_pgf_cal(1);  // if the RX calibration routine fails the device receiver performance will be severely affected,the application should reset and try again
	return error;
}

#define SEL_CHANNEL5				(5)
#define SEL_CHANNEL9				(9)

//vals
#define TXRXSWITCH_TX				0x01011100
#define TXRXSWITCH_AUTO				0x1C000000

void dwt_wrap_enable_rf_tx(uint32_t channel,uint8_t switch_control) {
	//Turn on TX LDOs
	dwt_or32bitoffsetreg(LDO_CTRL_ID,0,(LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK | LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK));
	dwt_or32bitoffsetreg(LDO_CTRL_ID,0,(LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK |LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK |LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK |LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK));
	//Enable RF blocks for TX (configure RF_ENABLE_ID reg)
	if(channel==SEL_CHANNEL5) {
		dwt_or32bitoffsetreg(RF_ENABLE_ID,0,(RF_ENABLE_TX_SW_EN_BIT_MASK|RF_ENABLE_TX_CH5_BIT_MASK|RF_ENABLE_TX_EN_BIT_MASK|RF_ENABLE_TX_EN_BUF_BIT_MASK|RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	}else{
		dwt_or32bitoffsetreg(RF_ENABLE_ID,0,(RF_ENABLE_TX_SW_EN_BIT_MASK|RF_ENABLE_TX_EN_BIT_MASK|RF_ENABLE_TX_EN_BUF_BIT_MASK|RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	}
	if (switch_control)	{
		//configure the TXRX switch for TX mode
		dwt_write32bitoffsetreg(RF_SWITCH_CTRL_ID,0x0,TXRXSWITCH_TX);
	}
}

void dwt_wrap_enable_rftx_blocks(uint32_t channel) {
	if(channel==SEL_CHANNEL5) {
		dwt_or32bitoffsetreg(RF_CTRL_MASK_ID,0,(RF_ENABLE_TX_SW_EN_BIT_MASK|RF_ENABLE_TX_CH5_BIT_MASK|RF_ENABLE_TX_EN_BIT_MASK|RF_ENABLE_TX_EN_BUF_BIT_MASK|RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	}else
	if(channel==SEL_CHANNEL9) {
		dwt_or32bitoffsetreg(RF_CTRL_MASK_ID,0,(RF_ENABLE_TX_SW_EN_BIT_MASK|RF_ENABLE_TX_EN_BIT_MASK|RF_ENABLE_TX_EN_BUF_BIT_MASK|RF_ENABLE_TX_BIAS_EN_BIT_MASK));
	}
}

void dwt_wrap_disable_rftx_blocks() {
    dwt_write32bitoffsetreg(RF_CTRL_MASK_ID,0,0x00000000);
}

void dwt_wrap_disable_rf_tx(uint8_t switch_config) {
	//Turn off TX LDOs
	dwt_write32bitoffsetreg(LDO_CTRL_ID,0,0x00000000);
	//Disable RF blocks for TX (configure RF_ENABLE_ID reg)
	dwt_write32bitoffsetreg(RF_ENABLE_ID,0,0x00000000);
	if(switch_config) {
		//Restore the TXRX switch to auto
		dwt_write32bitoffsetreg(RF_SWITCH_CTRL_ID,0x0,TXRXSWITCH_AUTO);
	}
}

uint8_t dwt_wrap_calcbandwidthadj(uint16_t target_count,uint8_t channel) {
	// Force system clock to FOSC/4 and TX clocks on and enable RF blocks
	dwt_wrap_force_clocks(FORCE_CLK_SYS_TX);
	dwt_wrap_enable_rf_tx(channel,0);
	dwt_wrap_enable_rftx_blocks(channel);

	// Write to the PG target before kicking off PG auto-cal with given target value
	dwt_write16bitoffsetreg(PG_CAL_TARGET_ID,0x0,target_count&PG_CAL_TARGET_TARGET_BIT_MASK);
	// Run PG count cal
	dwt_or8bitoffsetreg(PGC_CTRL_ID,0x0,(uint8_t)(PGC_CTRL_PGC_START_BIT_MASK|PGC_CTRL_PGC_AUTO_CAL_BIT_MASK));
	// Wait for calibration to complete
	while (dwt_read8bitoffsetreg(PGC_CTRL_ID,0)&PGC_CTRL_PGC_START_BIT_MASK);

	//Restore clocks to AUTO and turn off TX blocks
	dwt_wrap_disable_rftx_blocks();
	dwt_wrap_disable_rf_tx(0);
	dwt_wrap_force_clocks(FORCE_CLK_AUTO);

	return  (dwt_read8bitoffsetreg(TX_CTRL_HI_ID,0)&TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);
}

void dwt_wrap_configuretxrf(dwt_txconfig_t *config) {
	if (config->PGcount==0) {
		// Configure RF TX PG_DELAY
		dwt_write8bitoffsetreg(TX_CTRL_HI_ID,0,config->PGdly);
	}else{
		uint8_t channel=5;
		if (dwt_read8bitoffsetreg(CHAN_CTRL_ID,0)&0x1) {
			channel=9;
		}
		dwt_wrap_calcbandwidthadj(config->PGcount,channel);
	}
	// Configure TX power
	dwt_write32bitreg(TX_POWER_ID,config->power);
}

void dwt_wrap_setrxantennadelay(uint16_t antennaDly){
	uint8_t header[1]={0x9c};
	uint8_t write[2];
	*(uint16_t*)write=antennaDly;
	writetospi(sizeof(header),header,sizeof(write),write);
	// dwt_setrxantennadelay(antennaDly);
}

void dwt_wrap_setrxaftertxdelay(uint32_t rxDelayTime) {
	uint32_t val=dwt_read32bitreg(ACK_RESP_ID);		// Read ACK_RESP_T_ID register
	val&=(~ACK_RESP_W4R_TIM_BIT_MASK);				// Clear the timer (19:0)
	val|=(rxDelayTime&ACK_RESP_W4R_TIM_BIT_MASK);	// In UWB microseconds (e.g. turn the receiver on 20uus after TX)
	dwt_write32bitoffsetreg(ACK_RESP_ID,0,val);
}

void dwt_wrap_setrxtimeout(uint32_t time) {
	if(time>0) {
		dwt_write32bitoffsetreg(RX_FWTO_ID,0,time);
		dwt_or16bitoffsetreg(SYS_CFG_ID,0,SYS_CFG_RXWTOE_BIT_MASK);					// set the RX FWTO bit
	}else{
		dwt_and16bitoffsetreg(SYS_CFG_ID,0,(uint16_t)(~SYS_CFG_RXWTOE_BIT_MASK));	// clear the RX FWTO bit
	}
}

void dwt_wrap_setlnapamode(uint8_t lna_pa) {
	uint32_t gpio_mode = dwt_read32bitreg(GPIO_MODE_ID);
	gpio_mode&=(~(GPIO_MODE_MSGP0_MODE_BIT_MASK|GPIO_MODE_MSGP1_MODE_BIT_MASK|GPIO_MODE_MSGP4_MODE_BIT_MASK|GPIO_MODE_MSGP5_MODE_BIT_MASK|GPIO_MODE_MSGP6_MODE_BIT_MASK)); //clear GPIO 4,5,6,configuration
	if(lna_pa&DWT_LNA_ENABLE) {
		gpio_mode|=GPIO_PIN6_EXTRX;
	}
	if(lna_pa&DWT_PA_ENABLE) {
		gpio_mode|=(GPIO_PIN4_EXTDA|GPIO_PIN5_EXTTX);
	}
	if(lna_pa&DWT_TXRX_EN) {
		gpio_mode|=(GPIO_PIN0_EXTTXE|GPIO_PIN1_EXTRXE);
	}
	dwt_write32bitreg(GPIO_MODE_ID,gpio_mode);
}

void dwt_wrap_writesysstatuslo(uint32_t mask){
	uint8_t header[2]={0xc1,0x10};
	uint8_t write[4];
	*(uint32_t*)write=mask;
	writetospi(sizeof(header),header,sizeof(write),write);
	// dwt_writesysstatuslo(mask);
}

#define REG_DIRECT_OFFSET_MAX_LEN		(127)
#define TX_BUFFER_MAX_LEN				(1024)
#define PMSC_CTRL0_PLL2_SEQ_EN			0x01000000UL		// Enable PLL2 on/off sequencing by SNIFF mode
#define TX_BUFFER_ID					0x140000			// Transmit Data Buffer

int dwt_wrap_writetxdata(uint16_t txDataLength,uint8_t *txDataBytes,uint16_t txBufferOffset) {
	if ((txBufferOffset+txDataLength)<TX_BUFFER_MAX_LEN) {
		if(txBufferOffset<=REG_DIRECT_OFFSET_MAX_LEN) {
			// Directly write the data to the IC TX buffer
			dwt3000_writetodevice(TX_BUFFER_ID,txBufferOffset,txDataLength,txDataBytes);
		}else{
			// Program the indirect offset register A for specified offset to TX buffer
			dwt_write32bitreg(INDIRECT_ADDR_A_ID,(TX_BUFFER_ID>>16));
			dwt_write32bitreg(ADDR_OFFSET_A_ID,txBufferOffset);
			// Indirectly write the data to the IC TX buffer
			dwt3000_writetodevice(INDIRECT_POINTER_A_ID,0,txDataLength,txDataBytes);
		}
		return DWT_SUCCESS;
	}
	return DWT_ERROR;
}

#define DWT_TX_BUFF_OFFSET_ADJUST  128 // TX buffer offset adjustment when txBufferOffset > 127

void dwt_wrap_writetxfctrl(uint16_t txFrameLength,uint16_t txBufferOffset,uint8_t ranging) {
	uint32_t reg32;
	//DW3000/3700 - if offset is > 127,128 needs to be added before data is written,this will be subtracted internally
	//prior to writing the data
	if(txBufferOffset <= 127) {
		// Write the frame length to the TX frame control register
		reg32=txFrameLength|((uint32_t)(txBufferOffset) << TX_FCTRL_TXB_OFFSET_BIT_OFFSET)|((uint32_t)ranging << TX_FCTRL_TR_BIT_OFFSET);
		dwt_modify32bitoffsetreg(TX_FCTRL_ID,0,(uint32_t)~(TX_FCTRL_TXB_OFFSET_BIT_MASK|TX_FCTRL_TR_BIT_MASK|TX_FCTRL_TXFLEN_BIT_MASK),reg32);
	}else{
		// Write the frame length to the TX frame control register
		reg32=txFrameLength|((uint32_t)(txBufferOffset + DWT_TX_BUFF_OFFSET_ADJUST) << TX_FCTRL_TXB_OFFSET_BIT_OFFSET)|((uint32_t)ranging << TX_FCTRL_TR_BIT_OFFSET);
		dwt_modify32bitoffsetreg(TX_FCTRL_ID,0,(uint32_t)~(TX_FCTRL_TXB_OFFSET_BIT_MASK|TX_FCTRL_TR_BIT_MASK|TX_FCTRL_TXFLEN_BIT_MASK),reg32);
		reg32=dwt_read8bitoffsetreg(SAR_CTRL_ID,0); //DW3000/3700 - need to read this to load the correct TX buffer offset value
	}
}

// SYS_STATE_LO register errors
#define DW_SYS_STATE_TXERR				0xD0000					// TSE is in TX but TX is in IDLE in SYS_STATE_LO register

// brief This call initiates the transmission,input parameter indicates which TX mode is used see below
// input parameters:
// param mode -  if mode=DWT_START_TX_IMMEDIATE - immediate TX (no response expected)
//              if mode=DWT_START_TX_DELAYED - delayed TX (no response expected)  at specified time (time in DX_TIME register)
//              if mode=DWT_START_TX_DLY_REF - delayed TX (no response expected)  at specified time (time in DREF_TIME register + any time in DX_TIME register)
//              if mode=DWT_START_TX_DLY_RS  - delayed TX (no response expected)  at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
//              if mode=DWT_START_TX_DLY_TS  - delayed TX (no response expected)  at specified time (time in TX_TIME_LO register + any time in DX_TIME register)
//              if mode=DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED - immediate TX (response expected - so the receiver will be automatically turned on after TX is done)
//              if mode=DWT_START_TX_DELAYED/DLY_*| DWT_RESPONSE_EXPECTED - delayed TX (response expected - so the receiver will be automatically turned on after TX is done)
//              if mode=DWT_START_TX_CCA - Send the frame if no preamble detected within PTO time
//              if mode=DWT_START_TX_CCA |DWT_RESPONSE_EXPECTED - Send the frame if no preamble detected within PTO time and then enable RX*
// output parameters
// 				returns DWT_SUCCESS for success,or DWT_ERROR for error (e.g. a delayed transmission will be cancelled if the delayed time has passed)
int dwt_wrap_starttx(uint8_t mode) {
	int retval=DWT_SUCCESS ;
	uint16_t checkTxOK=0 ;
	uint32_t sys_state;
	if((mode&DWT_START_TX_DELAYED) || (mode&DWT_START_TX_DLY_REF) || (mode&DWT_START_TX_DLY_RS) || (mode&DWT_START_TX_DLY_TS)) {
		if(mode&DWT_START_TX_DELAYED) {// delayed TX
			if(mode&DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_W4R);
			}else{
				dwt_writefastCMD(CMD_DTX);
			}
		}else
		if(mode&DWT_START_TX_DLY_RS) { // delayed TX WRT RX timestamp
			if(mode&DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_RS_W4R);
			}else{
				dwt_writefastCMD(CMD_DTX_RS);
			}
		}else
		if(mode&DWT_START_TX_DLY_TS) { // delayed TX WRT TX timestamp
			if(mode&DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_TS_W4R);
			}else{
				dwt_writefastCMD(CMD_DTX_TS);
			}
		}else{ // delayed TX WRT reference time
			if(mode&DWT_RESPONSE_EXPECTED) {
				dwt_writefastCMD(CMD_DTX_REF_W4R);
			}else{
				dwt_writefastCMD(CMD_DTX_REF);
			}
		}
		checkTxOK=dwt_read8bitoffsetreg(SYS_STATUS_ID,3);		// Read at offset 3 to get the upper 2 bytes out of 5
		if((checkTxOK&(SYS_STATUS_HPDWARN_BIT_MASK>>24))==0) {	// Transmit Delayed Send set over Half a Period away.
			sys_state=dwt_read32bitreg(SYS_STATE_LO_ID);
			if(sys_state==DW_SYS_STATE_TXERR) {
				dwt_writefastCMD(CMD_TXRXOFF);
				retval=DWT_ERROR ; // Failed !
			}else{
				retval=DWT_SUCCESS ; // All okay
			}
		}else{
			uprintf("checkTxOK %d\n",checkTxOK);
			dwt_writefastCMD(CMD_TXRXOFF);
			retval=DWT_ERROR ; // Failed! optionally could return error,and still send the frame at indicated time,then if the application want to cancel the sending this can be done in a separate command.
		}
	}else
	if(mode&DWT_START_TX_CCA) {
		if(mode&DWT_RESPONSE_EXPECTED) {
			dwt_writefastCMD(CMD_CCA_TX_W4R);
		}else{
			dwt_writefastCMD(CMD_CCA_TX);
		}
	}else{
		if(mode&DWT_RESPONSE_EXPECTED) {
			dwt_writefastCMD(CMD_TX_W4R);
		}else{
			dwt_writefastCMD(CMD_TX);
		}
	}
	return retval;
}

#define RX_BUFFER_MAX_LEN			(1023)

#define RX_BUFFER_0_ID				0x120000	// Default Receive Data Buffer (and the 1st of the double buffer set)
#define RX_BUFFER_1_ID				0x130000	// 2nd Receive Data Buffer (when operating in double buffer mode)

void dwt_wrap_readrxdata(uint8_t *buffer,uint16_t length,uint16_t rxBufferOffset) {
	uint32_t  rx_buff_addr;
	if (pdw3000local->dblbuffon == DBL_BUFF_ACCESS_BUFFER_1) {					//if the flag is 0x3 we are reading from RX_BUFFER_1
		rx_buff_addr=RX_BUFFER_1_ID;
	}else{																		// reading from RX_BUFFER_0 - also when non-double buffer mode
		rx_buff_addr=RX_BUFFER_0_ID;
	}
	if ((rxBufferOffset + length) <= RX_BUFFER_MAX_LEN) {
		if(rxBufferOffset <= REG_DIRECT_OFFSET_MAX_LEN) {						// Directly read data from the IC to the buffer
			dwt3000_readfromdevice(rx_buff_addr,rxBufferOffset,length,buffer);
		}else{
			dwt_write32bitreg(INDIRECT_ADDR_A_ID,(rx_buff_addr >> 16) );		// Program the indirect offset registers B for specified offset to RX buffer
			dwt_write32bitreg(ADDR_OFFSET_A_ID,  rxBufferOffset);
			dwt3000_readfromdevice(INDIRECT_POINTER_A_ID,0,length,buffer);	// Indirectly read data from the IC to the buffer
		}
	}
}

uint32_t dwt_wrap_readtxtimestamplo32() {
	return dwt_read32bitreg(TX_TIME_LO_ID); // Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

uint16_t dwt_wrap_getframelength(){
	uint8_t header[2]={0x41,0x30};
	uint8_t read[2];
	readfromspi(sizeof(header),header,sizeof(read),read);
	uint16_t fl=(uint16_t)read[0];
	// uint16_t fl=dwt_getframelength();
	// printf("frame length 0x%02x\n",fl);
	return fl;
}

void dwt_wrap_settxantennadelay(uint16_t txDelay) {
	// Set the TX antenna delay for auto TX timestamp adjustment
	dwt_write16bitoffsetreg(TX_ANTD_ID,0,txDelay);
}

uint32_t dwt_wrap_readrxtimestamplo32() {
	return dwt_read32bitreg(RX_TIME_0_ID); // Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5 byte timestamp
}

#define BUF1_CIA_DIAG_0						0x1800F4			//part of min set
#define BUF0_CIA_DIAG_0						0x18000C			//part of min set

// dwt_readclockoffset defines
#define B11_SIGN_EXTEND_TEST				(0x1000UL)
#define B11_SIGN_EXTEND_MASK				(0xE000UL)

int16_t dwt_wrap_readclockoffset() {
	uint16_t  regval=0 ;
	switch (pdw3000local->dblbuffon) {		//if the flag is non zero - we are either accessing RX_BUFFER_0 or RX_BUFFER_1
		case DBL_BUFF_ACCESS_BUFFER_1:		//!!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
			regval=dwt_read16bitoffsetreg(INDIRECT_POINTER_B_ID,(BUF1_CIA_DIAG_0-BUF1_RX_FINFO))&CIA_DIAG_0_COE_PPM_BIT_MASK;
			break;
		case DBL_BUFF_ACCESS_BUFFER_0:
			regval=dwt_read16bitoffsetreg(BUF0_CIA_DIAG_0,0)&CIA_DIAG_0_COE_PPM_BIT_MASK;
			break;
		default:
			regval=dwt_read16bitoffsetreg(CIA_DIAG_0_ID,0)&CIA_DIAG_0_COE_PPM_BIT_MASK;
			break;
	}
	if (regval&B11_SIGN_EXTEND_TEST) {
		regval|=B11_SIGN_EXTEND_MASK;		// sign extend bit #12 to the whole short
	}
	return (int16_t) regval ;
}

uint32_t dwt_wrap_readsysstatuslo() {
	uint8_t header[2]={0x41,0x10};
	uint8_t read[4];
	readfromspi(sizeof(header),header,sizeof(read),read);
	uint32_t sysstatuslo=*(uint32_t*)read;
	// uint32_t sysstatuslo=dwt_readsysstatuslo();
	return sysstatuslo;
}

uint32_t dwt_wrap_readsysstatushi() {
	EnableDebugPrintSPI();
	uint32_t ret=0;
	FATAL("dwt_readsysstatushi");
	// dwt_readsysstatushi();
	DisableDebugPrintSPI();
	return ret;
}

void wrap_waitforsysstatus(uint32_t*lo_result,uint32_t*hi_result,uint32_t lo_mask,uint32_t hi_mask) {
	uint32_t lo_result_tmp=0;
	uint32_t hi_result_tmp=0;
	// If a mask has been passed into the function for the system status register (lower 32-bits)
	if(lo_mask) {
		while (!((lo_result_tmp=dwt_wrap_readsysstatuslo())&(lo_mask))) {
			// If a mask value is set for the system status register (higher 32-bits)
			if(hi_mask) {
				// If mask value for the system status register (higher 32-bits) is found
				if((hi_result_tmp=dwt_wrap_readsysstatushi())&hi_mask) {
					break;
				}
			}
		}
	}else
	if(hi_mask){// if only a mask value for the system status register (higher 32-bits) is set
		while (!((hi_result_tmp=dwt_wrap_readsysstatushi())&(hi_mask))) { };
	}
	if(lo_result !=NULL) {
		*lo_result=lo_result_tmp;
	}
	if(hi_result !=NULL) {
		*hi_result=hi_result_tmp;
	}
}

void dwt_wrap_readrxtimestamp(uint8_t*timestamp) {
	switch (pdw3000local->dblbuffon) {				// check if in double buffer mode and if so which buffer host is currently accessing
		case DBL_BUFF_ACCESS_BUFFER_1:
			// !!! Assumes that Indirect pointer register B was already set. This is done in the dwt_setdblrxbuffmode when mode is enabled.
			dwt3000_readfromdevice(INDIRECT_POINTER_B_ID,BUF1_RX_TIME -BUF1_RX_FINFO,RX_TIME_RX_STAMP_LEN,timestamp);
			break;
		case DBL_BUFF_ACCESS_BUFFER_0:
			dwt3000_readfromdevice(BUF0_RX_TIME,0,RX_TIME_RX_STAMP_LEN,timestamp);
			break;
		default:
			dwt3000_readfromdevice(RX_TIME_0_ID,0,RX_TIME_RX_STAMP_LEN,timestamp); // Get the adjusted time of arrival
			break;
	}
}

uint64_t wrap_get_rx_timestamp_u64() {
	uint8_t ts_tab[5];
	uint64_t ts=0;
	int8_t i;
	dwt_wrap_readrxtimestamp(ts_tab);
	for (i=4; i >=0; i--) {
		ts <<=8;
		ts |=ts_tab[i];
	}
	return ts;
}

void wrap_resp_msg_set_ts(uint8_t*ts_field,const uint64_t ts) {
	uint8_t i;
	for(i=0;i<RESP_MSG_TS_LEN;i++) {
		ts_field[i]=(uint8_t)(ts>>(i*8));
	}
}

void wrap_resp_msg_get_ts(uint8_t*ts_field,uint32_t*ts) {		// Read a given timestamp value from the response message. In the timestamp fields of the response message,the least significant byte is at the lower address
	int i;
	*ts=0;
	for (i=0;i<RESP_MSG_TS_LEN;i++) {
		*ts+=(uint32_t)ts_field[i]<<(i*8);
	}
}

void enable_cia() {
  dwt_modify8bitoffsetreg(0xe0000,2,0xef,0); // clear MINDIAG bit (CIA_CONF_ID)
}

float nlos_prob_ipa(){
  float pr_nlos = 1.0f;

	// See Decawave application note: 'APS006 PART 3'
  const float cSIG_LVL_THRESHOLD=12.f; // signal threshold in dB
  const float cSIG_LVL_MIN_THRESHOLD=12.f*0.4f;
  const float cIP_MIN_THRESHOLD=3.3f; // index difference threshold first vs peak
  const float cIP_MAX_THRESHOLD=6.0f;
  const float cLOG_CONSTANT_C0=63.2f; // 10log10(2^21) = 63.2

	// 22 bits, throw away 2 fractional bits
	uint32_t F1=(dwt_read32bitoffsetreg(IP_DIAG_2_ID,0)&0x3fffff)>>2;
  uint32_t F2=(dwt_read32bitoffsetreg(IP_DIAG_3_ID,0)&0x3fffff)>>2;
  uint32_t F3=(dwt_read32bitoffsetreg(IP_DIAG_4_ID,0)&0x3fffff)>>2;
  uint32_t cirPower=dwt_read32bitoffsetreg(IP_DIAG_1_ID,0)&0x1ffff;
  uint32_t ampPower=F1*F1+F2*F2+F3*F3;

  float sl_diff_ip = 10.0f * log10f(cirPower * 1.0f/ampPower) + cLOG_CONSTANT_C0;

  if (sl_diff_ip > cSIG_LVL_THRESHOLD) {
      pr_nlos = 1.0f;
  } else if (sl_diff_ip > cSIG_LVL_MIN_THRESHOLD) {
      float a = 1.0f/(cSIG_LVL_THRESHOLD-cSIG_LVL_MIN_THRESHOLD);
      float b =-cSIG_LVL_MIN_THRESHOLD*a;
      pr_nlos = a * sl_diff_ip + b;
  } else {
      uint32_t index_fp_u32=dwt_read32bitoffsetreg(IP_DIAG_8_ID, 0);                     // 0:0+16 [10.6fp] index of first path
      uint32_t index_pp_u32=((dwt_read32bitoffsetreg(IP_DIAG_0_ID, 0) >> 21)&0x3ff)<<6;  // 21:21+10 index of largest preample amplitude

      float index_diff = (index_pp_u32 - index_fp_u32) / 64.0f; // [10.6fp] -> float, deca example code uses /32.0f - why ??
      if(index_diff>cIP_MAX_THRESHOLD) {
        pr_nlos = 1.0f;
      } else if(index_diff>cIP_MIN_THRESHOLD) {
          const float a = 1.0f/(cIP_MAX_THRESHOLD-cIP_MIN_THRESHOLD);
          const float b = -cIP_MIN_THRESHOLD*a;
          pr_nlos = a * index_diff + b;
      } else {
          pr_nlos = 0.0f;
      }
  }
  return pr_nlos;
}
