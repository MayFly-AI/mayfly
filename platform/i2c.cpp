#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <mutex>
#include "gpio.h"
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <gpiod.h>


#include "shared/misc.h"
#include "platform/i2c.h"

volatile int g_initCount=0;

int InitializeI2C(const char* idname) {
	uprintf("InitializeI2C Begin %s\n",idname);
	static int file=-1;
	LockI2C();
	g_initCount++;
	if(file!=-1) {
		uprintf("InitializeI2C End %s called multiple times. Ignore\n",idname);
		UnlockI2C();
		return file;
	}
	const char* bus="/dev/i2c-0";
	if((file=open(bus,O_RDWR))<0) {
		uprintf("Failed to open the bus.\n");
		exit(1);
	}
	UnlockI2C();
	uprintf("InitializeI2C End %s file handle 0x%08x\n",idname,file);
	return file;
}

std::mutex g_lock;
void LockI2C() {
	g_lock.lock();
}
void UnlockI2C() {
	g_lock.unlock();
}

void CloseI2C(int file) {
	if(!--g_initCount) {
		close(file);
	}
}
void BeginDeviceI2C(int file,uint8_t deviceId) {
	if(ioctl(file,I2C_SLAVE,deviceId)<0) {
		FATAL("Failed to acquire bus access and/or talk to slave.\n");
		return;
	}
}
void EndDeviceI2C(int file) {
}

void WriteToAddressI2C(int file,uint8_t deviceId,uint8_t address,uint8_t value) {
	LockI2C();
	if(file==-1)
		FATAL("WriteToAddressI2C file 0x%08x\n",file);
	BeginDeviceI2C(file,deviceId);
	WriteToAddressI2C(file,address,value);
	UnlockI2C();
}
int ReadFromAddressI2C(int file,uint8_t deviceId,uint8_t* buffer,uint8_t address,uint8_t byteSize) {
	LockI2C();
	if(file==-1)
		FATAL("ReadFromAddressI2C file 0x%08x\n",file);
	BeginDeviceI2C(file,deviceId);
	int sz=ReadFromAddressI2C(file,buffer,address,byteSize);
	UnlockI2C();
	return sz;
}

void WriteToAddressI2C(int file,uint8_t address,uint8_t value) {
	char config[2];
	config[0]=address;
	config[1]=value;
	//write(file,config,2);
	write(file,config,2);
}
int ReadFromAddressI2C(int file,uint8_t* buffer,uint8_t address,uint8_t byteSize) {
	char reg[1]={address};
	write(file,reg,1);
	size_t sz=read(file,buffer,byteSize);
	if(sz!=byteSize) {
		FATAL("ReadFromAddressI2C %d!=%d failed\n",sz,byteSize);
	}
	return (int)sz;
}
