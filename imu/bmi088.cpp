/*
* Based on Brian R Taylor Bolder Flight Systems
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "shared/types.h"
#include "shared/math.h"
#include "shared/misc.h"

#include "platform/i2c.h"
#include "bmi088.h"

#define G_ACC 9.807f
#define GET_FIELD(regname,value) (((value) & regname##_MASK)>>regname##_POS)
#define	SET_FIELD(regval,regname,value) ((regval & ~regname##_MASK)|(((value)<<regname##_POS) & regname##_MASK))

//void BeginDeviceI2C(int file,uint8_t address);
//void WriteToAddressI2C(int file,uint8_t address,uint8_t value);
//void ReadFromAddressI2C(int file,uint8_t* buffer,uint8_t address,uint8_t byteSize);

static void delay(unsigned int time_ms) {
	usleep(time_ms*1000);
}

//Accel
void Accel::WriteRegister(uint8_t subAddress,uint8_t data,bool wait) {
	WriteToAddressI2C(m_file,0x18,subAddress,data);

	//LockI2C(m_file);
	//BeginDeviceI2C(m_file,0x18);
	//WriteToAddressI2C(m_file,subAddress,data);
	//UnlockI2C(m_file);
	//uint8_t values[2];
	//values[0]=subAddress;
	//values[1]=data;
	//twi_writeTo(g_address,values,2,1,wait);
}
void Accel::ReadRegisters(uint8_t subAddress,uint8_t count,uint8_t* dest) {
	ReadFromAddressI2C(m_file,0x18,dest,subAddress,count);

	//LockI2C(m_file);
	//BeginDeviceI2C(m_file,0x18);
	//ReadFromAddressI2C(m_file,dest,subAddress,count);
	//UnlockI2C(m_file);
	//twi_writeTo(g_address,&subAddress,1,1,false);
	//twi_readFrom(g_address,dest,count,false);
}
bool Accel::isCorrectId1() {
	uint8_t readReg=0;
	ReadRegisters(ACC_CHIP_ID_ADDR,1,&readReg);
	return (GET_FIELD(ACC_CHIP_ID,readReg)==ACC_CHIP_ID) ? true : false;
}
bool Accel::SetMode(bool active) {
	uint8_t writeReg=0,readReg=0;
	uint8_t value=(active) ? ACC_ACTIVE_MODE_CMD : ACC_SUSPEND_MODE_CMD;
	writeReg=SET_FIELD(writeReg,ACC_PWR_CONF,value);
	WriteRegister(ACC_PWR_CONF_ADDR,writeReg);
	delay(5); // 5 ms wait after power mode changes
	ReadRegisters(ACC_PWR_CONF_ADDR,1,&readReg);
	return (readReg==writeReg) ? true : false;
}
bool Accel::SetPower(bool enable) {
	uint8_t writeReg=0,readReg=0;
	uint8_t value=(enable) ? ACC_ENABLE_CMD : ACC_DISABLE_CMD;
	writeReg=SET_FIELD(writeReg,ACC_PWR_CNTRL,value);
	WriteRegister(ACC_PWR_CNTRL_ADDR,writeReg);
	delay(5); // 5 ms wait after power mode changes
	ReadRegisters(ACC_PWR_CNTRL_ADDR,1,&readReg);
	return (readReg==writeReg) ? true : false;
}
void Accel::SoftReset() {
	uint8_t reg=0;
	reg=SET_FIELD(reg,ACC_SOFT_RESET,ACC_RESET_CMD);
	WriteRegister(ACC_SOFT_RESET_ADDR,reg,true);
	delay(50);
}
bool Accel::SetOdr(uint8_t odr,uint8_t bwp) {
	uint8_t writeReg=SET_FIELD(writeReg,ACC_ODR,(1<<7)|odr|bwp);
	WriteRegister(ACC_ODR_ADDR,writeReg);
	delay(1);
	uint8_t readReg=0;
	ReadRegisters(ACC_ODR_ADDR,1,&readReg);
	return (readReg==writeReg) ? true:false;
}
bool Accel::SetRange(uint8_t  range) {
	uint8_t writeReg=0,readReg=0;
	ReadRegisters(ACC_RANGE_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,ACC_RANGE,range);
	WriteRegister(ACC_RANGE_ADDR,writeReg);
	delay(1);
	ReadRegisters(ACC_RANGE_ADDR,1,&readReg);
	if(readReg==writeReg) {
		switch(range) {
			case ACC_RANGE_3G: {
				m_accelRangeMSS=3.0f*G_ACC;
				break;
			}
			case ACC_RANGE_6G: {
				m_accelRangeMSS=6.0f*G_ACC;
				break;
			}
			case ACC_RANGE_12G: {
				m_accelRangeMSS=12.0f*G_ACC;
				break;
			}
			case ACC_RANGE_24G: {
				m_accelRangeMSS=24.0f*G_ACC;
				break;
			}      
		}
		return true;
	} else {
		return false;
	}
}
bool Accel::PinModeInt1(uint8_t pin_io,uint8_t pin_mode,uint8_t active_lvl) {
	uint8_t writeReg=0,readReg=0;
	ReadRegisters(ACC_INT1_IO_CTRL_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,ACC_INT1_IO_CTRL,(pin_io|pin_mode|active_lvl));
	WriteRegister(ACC_INT1_IO_CTRL_ADDR,writeReg);
	delay(1);
	ReadRegisters(ACC_INT1_IO_CTRL_ADDR,1,&readReg);
	return (readReg==writeReg) ? true : false;  
}
bool Accel::PinModeInt2(uint8_t pin_io,uint8_t pin_mode,uint8_t active_lvl) {
	uint8_t writeReg=0,readReg=0;
	ReadRegisters(ACC_INT2_IO_CTRL_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,ACC_INT2_IO_CTRL,(pin_io|pin_mode|active_lvl));
	WriteRegister(ACC_INT2_IO_CTRL_ADDR,writeReg);
	delay(1);
	ReadRegisters(ACC_INT2_IO_CTRL_ADDR,1,&readReg);
	return (readReg==writeReg)?true:false;
}
bool Accel::MapDrdyInt1(bool enable) {
	uint8_t writeReg=0,readReg=0;
	ReadRegisters(ACC_INT1_DRDY_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,ACC_INT1_DRDY,enable);
	WriteRegister(ACC_INT1_DRDY_ADDR,writeReg);
	delay(1);
	ReadRegisters(ACC_INT1_DRDY_ADDR,1,&readReg);
	return (readReg==writeReg) ? true : false;  
}
bool Accel::MapDrdyInt2(bool enable) {
	uint8_t writeReg=0, readReg=0;
	ReadRegisters(ACC_INT2_DRDY_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,ACC_INT2_DRDY,enable);
	WriteRegister(ACC_INT2_DRDY_ADDR,writeReg);
	delay(1);
	ReadRegisters(ACC_INT2_DRDY_ADDR,1,&readReg);
	return (readReg==writeReg) ? true:false;
}
bool Accel::Read(V3* acc,uint32_t* timeCounter,float* temperature) {
	const int16_t tX[3]={1,0,0};
	const int16_t tY[3]={0,-1,0};
	const int16_t tZ[3]={0,0,-1};
	int16_t accel[3];
	uint8_t _buffer[9];
	ReadRegisters(ACC_ACCEL_DATA_ADDR,9,_buffer);
	accel[0]=(_buffer[1]<<8)|_buffer[0];
	accel[1]=(_buffer[3]<<8)|_buffer[2];
	accel[2]=(_buffer[5]<<8)|_buffer[4];
	acc->x=(float) (accel[0]*tX[0]+accel[1]*tX[1]+accel[2]*tX[2])/32768.0f*m_accelRangeMSS;
	acc->y=(float) (accel[0]*tY[0]+accel[1]*tY[1]+accel[2]*tY[2])/32768.0f*m_accelRangeMSS;
	acc->z=(float) (accel[0]*tZ[0]+accel[1]*tZ[1]+accel[2]*tZ[2])/32768.0f*m_accelRangeMSS;
	//time data
	uint32_t current_time_counter=(_buffer[8]<<16)|(_buffer[7]<<8)|_buffer[6];
	*timeCounter=current_time_counter-m_prevTimeCounter;
	m_prevTimeCounter=current_time_counter;
	// temperature data
	ReadRegisters(ACC_TEMP_DATA_ADDR,2,_buffer);
	int16_t temp_int11;
	uint16_t temp_uint11=(_buffer[0]*8)+(_buffer[1]/32);
	if (temp_uint11 > 1023) {
		temp_int11=temp_uint11-2048;
	} else {
		temp_int11=temp_uint11;
	}
	*temperature=(float) temp_int11*0.125f+23.0f;
	return true;
}
bool Accel::SelfTest() {
	uint8_t writeReg = 0;
	float accel_pos_mg[3], accel_neg_mg[3];
	// set 24G range
	SetRange(ACC_RANGE_24G);
	// set 1.6 kHz ODR, 4x oversampling
	SetOdr(ACC_ODR_1600HZ,ACC_NORMAL);
	// wait >2 ms
	delay(3);
	// enable self test, positive polarity
	writeReg=SET_FIELD(writeReg,ACC_SELF_TEST,ACC_POS_SELF_TEST);
	WriteRegister(ACC_SELF_TEST_ADDR,writeReg);
	// wait >50 ms
	delay(51);
	// read self test values
	V3 acc;
	uint32_t timeCounter;
	float temperature;
	Read(&acc,&timeCounter,&temperature);
	for (uint8_t i = 0; i < 3; i++) {
		accel_pos_mg[i]=acc[i]/G_ACC*1000.0f;
	}
	// enable self test, negative polarity
	writeReg = SET_FIELD(writeReg,ACC_SELF_TEST,ACC_NEG_SELF_TEST);
	WriteRegister(ACC_SELF_TEST_ADDR,writeReg);
	// wait >50 ms
	delay(51);
	// read self test values
	Read(&acc,&timeCounter,&temperature);
	for (uint8_t i = 0; i < 3; i++) {
		accel_neg_mg[i]=acc[i]/G_ACC*1000.0f;
	}
	// disable self test
	writeReg = SET_FIELD(writeReg,ACC_SELF_TEST,ACC_DIS_SELF_TEST);
	WriteRegister(ACC_SELF_TEST_ADDR,writeReg);
	// wait >50 ms
	delay(51); 
	// check self test results
	if((fabs(accel_pos_mg[0]-accel_neg_mg[0])>=1000) && (fabs(accel_pos_mg[1]-accel_neg_mg[1]) >= 1000) && (fabs(accel_pos_mg[2]-accel_neg_mg[2])>=500)) {
		return true;
	}else{
		return false;
	}
}
bool Accel::Begin() {
	if(!isCorrectId1()) {
		return false;
	}
	SoftReset();
	if(!SetPower(true)) {
		return false;
	}
	if(!SetMode(true)) {
		return false;
	}
	if(!SelfTest()) {
		return false;
	}

	SoftReset();
	if(!SetPower(true)) {
		return false;
	}
	if(!SetMode(true)) {
		return false;
	}

	delay(50);
	if (!SetRange(ACC_RANGE_24G)) {
		return false;
	}
	if(!SetOdr(ACC_ODR_100HZ,ACC_NORMAL)) {
		return false;
	}
	return true;
}








//Gyro
void Gyro::WriteRegister(uint8_t subAddress,uint8_t data,bool wait) {
	LockI2C();
	BeginDeviceI2C(m_file,0x68);
	WriteToAddressI2C(m_file,subAddress,data);
	UnlockI2C();
	//uint8_t values[2];
	//values[0]=subAddress;
	//values[1]=data;
	//twi_writeTo(g_address,values,2,1,wait);
}
void Gyro::ReadRegisters(uint8_t subAddress,uint8_t count,uint8_t* dest) {
	LockI2C();
	BeginDeviceI2C(m_file,0x68);
	ReadFromAddressI2C(m_file,dest,subAddress,count);
	UnlockI2C();
	//twi_writeTo(g_address,&subAddress,1,1,false);
	//twi_readFrom(g_address,dest,count,false);
}
void Gyro::ValidateId() {
	uint8_t readReg=0;
	ReadRegisters(GYRO_CHIP_ID_ADDR,1,&readReg);
	uint8_t id=GET_FIELD(GYRO_CHIP_ID,readReg);
	if(id!=GYRO_CHIP_ID) {
		FATAL("GYRO_CHIP_ID %d!=%d\n",GYRO_CHIP_ID,id);
	}
	uprintf("gyro id %d\n",id);
}
void Gyro::SoftReset() {
	uint8_t reg=0;
	reg=SET_FIELD(reg,GYRO_SOFT_RESET,GYRO_RESET_CMD);
	WriteRegister(GYRO_SOFT_RESET_ADDR,reg);
	delay(50);
}
bool Gyro::SetRange(uint8_t range) {
	uint8_t writeReg=0,readReg=0;
	writeReg=SET_FIELD(writeReg,GYRO_RANGE,range);
	WriteRegister(GYRO_RANGE_ADDR,writeReg);
	delay(1);
	ReadRegisters(GYRO_RANGE_ADDR,1,&readReg);
	if(readReg==writeReg) {
		const float D2R=M_PI/180.0f;
		switch (range) {
			case GYRO_RANGE_125DPS: {
				m_gyroRangeRads=125.0f*D2R;
				break;
			}
			case GYRO_RANGE_250DPS: {
				m_gyroRangeRads=250.0f*D2R;
				break;
			}
			case GYRO_RANGE_500DPS: {
				m_gyroRangeRads=500.0f*D2R;
				break;
			}
			case GYRO_RANGE_1000DPS: {
				m_gyroRangeRads=1000.0f*D2R;
				break;
			}
			case GYRO_RANGE_2000DPS: {
				m_gyroRangeRads=2000.0f*D2R;
				break;
			}       
		}
		return true;
	}else{
		return false;
	}
}

//enables the new data interrupt
bool Gyro::SetDataInterupt(bool enable) {
	uint8_t writeReg=0,readReg=0;
	uint8_t value=(enable) ? GYRO_ENABLE_DRDY_INT : GYRO_DIS_DRDY_INT;
	writeReg=SET_FIELD(writeReg,GYRO_INT_CNTRL,value);
	WriteRegister(GYRO_INT_CNTRL_ADDR,writeReg);
	delay(1); 
	ReadRegisters(GYRO_INT_CNTRL_ADDR,1,&readReg);
	return (readReg==writeReg) ? true : false;  
}
bool Gyro::SetOdr(uint8_t odr) {
	uint8_t writeReg=0,readReg=0;
	writeReg=SET_FIELD(writeReg,GYRO_ODR,odr);
	WriteRegister(GYRO_ODR_ADDR,writeReg);
	delay(1);
	ReadRegisters(GYRO_ODR_ADDR,1,&readReg);
	return (readReg==writeReg) ? true : false;
}

void Gyro::Read(V3* rads) {
	const int16_t tX[3]={1,0,0};
	const int16_t tY[3]={0,-1,0};
	const int16_t tZ[3]={0,0,-1};
	uint8_t _buffer[6];
	ReadRegisters(GYRO_DATA_ADDR,6,_buffer);
	int16_t gyro[3];
	gyro[0]=(_buffer[1]<<8)|_buffer[0];
	gyro[1]=(_buffer[3]<<8)|_buffer[2];
	gyro[2]=(_buffer[5]<<8)|_buffer[4];
	rads->x=(float)(gyro[0]*tX[0]+gyro[1]*tX[1]+gyro[2]*tX[2])/32767.0f*m_gyroRangeRads;
	rads->y=(float)(gyro[0]*tY[0]+gyro[1]*tY[1]+gyro[2]*tY[2])/32767.0f*m_gyroRangeRads;
	rads->z=(float)(gyro[0]*tZ[0]+gyro[1]*tZ[1]+gyro[2]*tZ[2])/32767.0f*m_gyroRangeRads;
}
bool Gyro::PinModeInt3(uint8_t pin_mode,uint8_t active_lvl) {
	uint8_t writeReg=0, readReg=0;
	ReadRegisters(GYRO_INT3_IO_CTRL_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,GYRO_INT3_IO_CTRL,(pin_mode|active_lvl));
	WriteRegister(GYRO_INT3_IO_CTRL_ADDR,writeReg);
	delay(1);
	ReadRegisters(GYRO_INT3_IO_CTRL_ADDR,1,&readReg);
	return (readReg==writeReg) ? true:false;
}
//sets the Int4 pin configuration
bool Gyro::PinModeInt4(uint8_t pin_mode,uint8_t active_lvl) {
	uint8_t writeReg=0,readReg=0;
	ReadRegisters(GYRO_INT4_IO_CTRL_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,GYRO_INT4_IO_CTRL,(pin_mode|active_lvl));
	WriteRegister(GYRO_INT4_IO_CTRL_ADDR,writeReg);
	delay(1);
	ReadRegisters(GYRO_INT4_IO_CTRL_ADDR,1,&readReg);
	return (readReg==writeReg) ? true:false;
}
//maps the data ready signal to the Int4 pin
bool Gyro::MapDrdyInt4(bool enable) {
	uint8_t writeReg=0,readReg=0;
	ReadRegisters(GYRO_INT4_DRDY_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,GYRO_INT4_DRDY,enable);
	WriteRegister(GYRO_INT4_DRDY_ADDR,writeReg);
	delay(1);
	ReadRegisters(GYRO_INT4_DRDY_ADDR,1,&readReg);
	return (readReg==writeReg) ? true:false;
}
//maps the data ready signal to the Int3 pin
bool Gyro::MapDrdyInt3(bool enable) {
	uint8_t writeReg=0, readReg=0;
	ReadRegisters(GYRO_INT3_DRDY_ADDR,1,&readReg);
	writeReg=SET_FIELD(readReg,GYRO_INT3_DRDY,enable);
	WriteRegister(GYRO_INT3_DRDY_ADDR,writeReg);
	delay(1);
	ReadRegisters(GYRO_INT3_DRDY_ADDR,1,&readReg);
	return (readReg==writeReg) ? true:false;
}
bool Gyro::Begin() {
	ValidateId();
	SoftReset();
	if(!SetRange(GYRO_RANGE_2000DPS)) {
		uprintf("SetRange failed\n");
		return false;
	}
	if(!SetDataInterupt(true)) {
		uprintf("SetDataInterupt failed\n");
		return false;
	}
	if(!SetOdr(GYRO_ODR_2000HZ_BW_532HZ)){
		uprintf("SetDataInterupt failed\n");
		return false;
	}
	return true;
}
