#include "imu.h"

#if ENABLE_LIBGPIOD
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<chrono>
#include<thread>
#include<atomic>
#include<assert.h>
#include<algorithm>

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"

#include "bmi088.h"

#include <poll.h>

#include "platform/gpio.h"
#include "platform/i2c.h"

int SetupTest(int file,uint8_t address, IMUOutputReadyCallback cb, int freq);

void ReadIMU(IMUOutputReadyCallback cb, int freq) {
	ConfigGPIO cfg;
	InitGPIO(&cfg);
	int file=InitializeI2C("BMI088");
	SetupTest(file,0x18,cb,freq);
	CloseI2C(file);
}

int SetupTest(int file,uint8_t address, IMUOutputReadyCallback cb, int freq) {
	Accel accel(file);
	if(!accel.Begin()) {
		FATAL("Accel::Begin failed");
	}
	Gyro gyro(file);
	if(!gyro.Begin()) {
		FATAL("Gyro::Begin failed");
	}
	if(freq==100) {
		if(!accel.SetOdr(ACC_ODR_100HZ,ACC_NORMAL)) {
			return -4;
		}
	}
	if(freq==50) {
		if(!accel.SetOdr(ACC_ODR_50HZ,ACC_NORMAL)) {
			return -4;
		}
	}
	if(freq==25) {
		if(!accel.SetOdr(ACC_ODR_25HZ,ACC_NORMAL)) {
			return -4;
		}
	}
	if(freq==12) {
		if(!accel.SetOdr(ACC_ODR_12HZ,ACC_NORMAL)) {
			return -4;
		}
	}
	if(!accel.PinModeInt1(ACC_INT_OUTPUT,ACC_INT_PUSHPULL,ACC_INT_LVL_HIGH)) {
		FATAL("PinModeInt1 failed");
	}
	if(!accel.MapDrdyInt1(true)) {
		FATAL("mapDrdyInt1 failed");
	}
	if(!gyro.SetOdr(GYRO_ODR_100HZ_BW_12HZ)) {
		FATAL("SetOutputDataRate failed");
	}
	//mapSync specify pin 4 to loop back the gyro interrupt
	if(!gyro.PinModeInt4(GYRO_INT_PUSHPULL,GYRO_INT_LVL_HIGH)) {
		FATAL("PinModeInt4 failed");
	}
	if(!gyro.MapDrdyInt4(true)) {
		FATAL("MapDrdyInt4 failed");
	}
	ConfigGPIO cfg;
	InitGPIO(&cfg);
	//int count=0;
	//uint64_t t0=0;//GetTimeEpochMicroseconds();
	uprintf("IMU enter data loop\n");
	while(true) {
		WaitForDataGPIO(&cfg,22);
		uint64_t t1=GetTimeEpochMicroseconds();
		//if(t1-t0>1000000) {
		//	uprintf("data %d!\n",count);
		//	count=0;
		//	t0=t1;
		//}else{
		//	count++;
		//}
		V3 acc;
		uint32_t timeCounter;
		float temperature;
		accel.Read(&acc,&timeCounter,&temperature);
		V3 rads;
		gyro.Read(&rads);
		cb(acc,rads,temperature,timeCounter,t1);
		//preciseSleep(1000000);
	}
	CloseGPIO(&cfg);
	/*
	if(!accel.setRange(Accel::RANGE_6G)) {
		FATAL("accel set range failed");
	}
	if(!gyro.SetRange(Gyro::GYRO_RANGE_500DPS)) {
		FATAL("SetRange failed");
	}
	// set the output data rate
	//if(!accel.SetOdr(Accel::ODR_400HZ_BW_145HZ)) {
	//	FATAL("SetOdr failed");
	//}
	if(!gyro.SetOutputDataRate(Gyro::ODR_400HZ_BW_47HZ)) {
		FATAL("SetOutputDataRate failed");
	}
	//mapSync specify whether to use pin 3 or pin 4 to loop back the gyro interrupt
	if(!gyro.PinModeInt3(GYRO_INT_PUSHPULL,GYRO_INT_LVL_HIGH)) {
		FATAL("PinModeInt3 failed");
	}
	if(!gyro.MapDrdyInt3(true)) {
		FATAL("MapDrdyInt3 failed");
	}
	//mapDrdy use pin 2 to indicate data ready, pin 1 will be used for gyro interrupt input

	if(!accel.PinModeInt1(ACC_INT_INPUT,ACC_INT_PUSHPULL,ACC_INT_LVL_HIGH)) {
		return false;
	}
	accel.WriteRegister(Accel::ACC_INT2_MAP_ADDR,Accel::ACC_INTA_ENABLE);
	if(!accel.PinModeInt2(ACC_INT_OUTPUT,ACC_INT_PUSHPULL,ACC_INT_LVL_HIGH)) {
		return false;
	}

	while(true) {
		uprintf("loop\n");
		preciseSleep(1000000);
	}

	while(true) {
		V3 rads;
		gyro.Read(&rads);
		uprintf("gyro %f,%f,%f\n",rads.x,rads.y,rads.z);
		preciseSleep(1000000);
	}

	if(!accel.PinModeInt1(ACC_INT_OUTPUT,ACC_INT_PUSHPULL,ACC_INT_LVL_HIGH)) {
		FATAL("PinModeInt1 failed");
	}
	if(!accel.mapDrdyInt1(true)) {
		FATAL("mapDrdyInt1 failed");
	}

	while(true) {
		V3 acc;
		uint32_t timeCounter;
		float temperature;
		accel.Read(&acc,&timeCounter,&temperature);
		uprintf("time %d acc %f,%f,%f temperature %f\n",timeCounter,acc.x,acc.y,acc.z,temperature);
		preciseSleep(1000000);
	}
	*/
	return 0;
}


#else
void ReadIMU(IMUOutputReadyCallback cb, int freq) { }
#endif

