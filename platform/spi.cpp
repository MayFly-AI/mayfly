#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<assert.h>

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <linux/spi/spidev.h>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"

#include "spi.h"
#ifdef LORT
int spi_fd=1337;

int spi_open(const char *device,spi_config_t config) {
	// Open block device
	int fd=open(device,O_RDWR);
	if(fd<0) {
		uprintf("spi_open open() failed\n");
		uprintf("Error no is : %d\n",errno);
		uprintf("Error description is : %s\n",strerror(errno));
		return -1;
	}
	// Set SPI_POL and SPI_PHA
	if(ioctl(fd,SPI_IOC_WR_MODE,&config.mode)<0) {
		return -1;
	}
	if(ioctl(fd,SPI_IOC_RD_MODE,&config.mode)<0) {
		return -1;
	}
	// Set bits per word
	if(ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&config.bits_per_word)<0) {
		return -1;
	}
	if(ioctl(fd,SPI_IOC_RD_BITS_PER_WORD,&config.bits_per_word)<0) {
		return -1;
	}
	// Set SPI speed
	if(ioctl(fd,SPI_IOC_WR_MAX_SPEED_HZ,&config.speed)<0) {
		return -1;
	}
	if(ioctl(fd,SPI_IOC_RD_MAX_SPEED_HZ,&config.speed)<0) {
		return -1;
	}
	// Return file descriptor
	return fd;
}

int spi_close(int fd) {
	return close(fd);
}

int spi_xfer(int fd,uint8_t *tx_buffer,uint8_t *rx_buffer,uint8_t len){
	struct spi_ioc_transfer spi_message[1];
	memset(spi_message,0,sizeof(spi_message));
	spi_message[0].rx_buf=(unsigned long)rx_buffer;
	spi_message[0].tx_buf=(unsigned long)tx_buffer;
	spi_message[0].len=len;
	return ioctl(fd,SPI_IOC_MESSAGE(1),spi_message);
}

int spi_read(int fd,uint8_t *rx_buffer,uint8_t rx_len){
	struct spi_ioc_transfer spi_message[1];
	memset(spi_message,0,sizeof(spi_message));
	spi_message[0].rx_buf=(unsigned long)rx_buffer;
	spi_message[0].len=rx_len;
	return ioctl(fd,SPI_IOC_MESSAGE(1),spi_message);
}

int spi_write(int fd,const uint8_t *tx_buffer,uint8_t tx_len){
	struct spi_ioc_transfer spi_message[1];
	memset(spi_message,0,sizeof(spi_message));
	spi_message[0].tx_buf=(unsigned long)tx_buffer;
	spi_message[0].len=tx_len;
	return ioctl(fd,SPI_IOC_MESSAGE(1),spi_message);
}

void rpi_spi_init() {
	// Start SPI
	spi_config_t spi_cfg;
	spi_cfg.mode=0;
	spi_cfg.bits_per_word=8;
	spi_cfg.speed=20000000;
	spi_cfg.delay=0;
	std::string spi_device("/dev/spidev0.0");
	spi_fd=spi_open(spi_device.c_str(),spi_cfg);
}





#endif//LORT
















/*

typedef struct {
    uint8_t mode;
    uint8_t bits_per_word;
    uint32_t speed;
    uint16_t delay;
} spi_config_t;
*/
int spi_open(const char *device, spi_config_t config) {
	// Open block device
	int fd=open(device, O_RDWR);
	if (fd < 0) {
		uprintf("spi_open open() failed\n");
		uprintf("Error no is : %d\n", errno);
		uprintf("Error description is : %s\n",strerror(errno));
		return -1;
	}
	// Set SPI_POL and SPI_PHA
	if (ioctl(fd, SPI_IOC_WR_MODE, &config.mode) < 0) {
		return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_MODE, &config.mode) < 0) {
		return -1;
	}
	// Set bits per word
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config.bits_per_word) < 0) {
		return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config.bits_per_word) < 0) {
		return -1;
	}
	// Set SPI speed
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config.speed) < 0) {
		return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config.speed) < 0) {
		return -1;
	}
	// Return file descriptor
	return fd;
}

int spi_close(int fd) {
    return close(fd);
}

int spi_xfer(int fd, uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t len){
    struct spi_ioc_transfer spi_message[1];
    memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].rx_buf = (unsigned long)rx_buffer;
    spi_message[0].tx_buf = (unsigned long)tx_buffer;
    spi_message[0].len = len;
    
    return ioctl(fd, SPI_IOC_MESSAGE(1), spi_message);
}

int spi_read(int fd, uint8_t *rx_buffer, uint8_t rx_len){
    struct spi_ioc_transfer spi_message[1];
    memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].rx_buf = (unsigned long)rx_buffer;
    spi_message[0].len = rx_len;
    
    return ioctl(fd, SPI_IOC_MESSAGE(1), spi_message);
}

int spi_write(int fd,const uint8_t *tx_buffer, uint8_t tx_len){
    struct spi_ioc_transfer spi_message[1];
    memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].tx_buf = (unsigned long)tx_buffer;
    spi_message[0].len = tx_len;
    
    return ioctl(fd, SPI_IOC_MESSAGE(1), spi_message);
}

int spi_fd=1337;

// Nordic board opens and closes spi everytime, we open SPI once
void rpi_spi_init() {
	//uprintf("rpi_spi_inif\n");
	// Start SPI
	spi_config_t spi_cfg;
	spi_cfg.mode = 0;
	spi_cfg.bits_per_word = 8;
	spi_cfg.speed = 2000000;
	spi_cfg.delay = 0;
	std::string spi_device("/dev/spidev0.0");
	spi_fd = spi_open(spi_device.c_str(),spi_cfg);
	//uprintf("spi_open. fd: %d\n",spi_fd);

    // Configure the chip select as an output pin that can be toggled
//	RPI_GPIO::setup_gpio(CS_pin,RPI_GPIO::PIN_MODE::OUTPUT,0);
//	RPI_GPIO::output_gpio(CS_pin,1);
}

#define MAX_PRINT_LEN 128

//#define DEBUG_SPI_PRINT
volatile static bool g_debugPrintSPI=false;
void EnableDebugPrintSPI() {
  g_debugPrintSPI=true;
}
void DisableDebugPrintSPI() {
  g_debugPrintSPI=false;
}

void PrintBuffer(const char* name,const uint8_t *dataBuffer,uint16_t dataLength) {
	if(!g_debugPrintSPI) {
		return;
	}
	if(dataLength>MAX_PRINT_LEN) {
		uprintf("%s data length %d > max print size %d\n",name,dataLength,MAX_PRINT_LEN);
		return;
	}
	if(dataLength<=0) {
		uprintf("%s %d\n",name,dataLength);
		return;
	}
	char buf[MAX_PRINT_LEN*5+1];
	char* p=buf;
	for(int i=0;i!=dataLength;i++) {
		int len=sprintf(p,"$%02x,",dataBuffer[i]);
		p+=len;
	}
	p[-1]=0;
	uprintf("%s %d %s\n",name,dataLength,buf);

}
void PrintCommand(const char* name,const uint8_t *dataBuffer,uint16_t dataLength) {
	if(!g_debugPrintSPI) {
		return;
	}
	uint8_t byte0=dataBuffer[0];
	uint8_t reg=(byte0>>1)&0x1f;
	bool RD_WR=byte0&0x80 ? true:false; //Read or write
	if(!(byte0&0x40)) {                 //Long or short/fast
		if(byte0&0x01) {                  //Fast or short command
			uprintf("%s FAST %s CMD $%02x\n",name,RD_WR?"WR":"RD",reg);
		}else{
			uprintf("%s SHORT %s REG $%02x\n",name,RD_WR?"WR":"RD",reg);
		}
	}else{
		uint8_t byte1=dataBuffer[1];
		bool M0=byte1&0x02;
		bool M1=byte1&0x01;
		int octet=0;
		if(M0==0 && M1==1)octet=1;
		if(M0==1 && M1==0)octet=2;
		if(M0==1 && M1==1)octet=4;
		uint8_t sub=((byte0&1)<<6)|((byte1>>2)&0x3f);
		uprintf("%s LONG %s M0=%d M1=%d REG=$%02x SUB=$%02x OCTET=%d\n",name,RD_WR?"WR":"RD",M0,M1,reg,sub,octet);
	}
}

int writetospi(uint16_t headerLength,const uint8_t* headerBuffer,uint16_t bodyLength,const uint8_t* bodyBuffer) {
#ifdef DEBUG_SPI_PRINT
	static int g_breakWriteCount=100000;
	PrintBuffer("writetospi header:",headerBuffer,headerLength);
	PrintCommand("writetospi:",headerBuffer,headerLength);
	PrintBuffer("writetospi body:",bodyBuffer,bodyLength);
	g_breakWriteCount--;
	if(!g_breakWriteCount) {
		FATAL("!g_breakWriteCount\n");
	}

	//if(g_debugPrintSPI) {
	//	uprintf("writetospi %d %d\n",headerLength,bodyLength);
	//}

#endif//DEBUG_SPI_PRINT

	//if(bodyLength==0) {
	//	int ret=spi_write(spi_fd,headerBuffer,headerLength);
	//	if(ret<0) {
	//		FATAL("error: ret %d\n",ret);
	//	}
	//}else{
		uint8_t buf[64];
		uint8_t buf1[64];
		memset(buf1,0,sizeof(buf1));
		memcpy(buf,headerBuffer,headerLength);
		memcpy(buf+headerLength,bodyBuffer,bodyLength);
		int ret=spi_xfer(spi_fd,buf,buf1,headerLength+bodyLength);
		if(ret<0) {
			FATAL("error: ret %d\n",ret);
		}
	//}
	return 0;
}

int readfromspi(uint16_t headerLength, uint8_t* headerBuffer, uint16_t readLength, uint8_t* readBuffer) {
#ifdef DEBUG_SPI_PRINT
	PrintBuffer("readfromspi header:",headerBuffer,headerLength);
	PrintCommand("readfromspi:",headerBuffer,headerLength);
	//if(g_debugPrintSPI) {
	//	uprintf("readfromspi %d %d\n",headerLength,readLength);
	//}
#endif//DEBUG_SPI_PRINT
	uint8_t buf[64];
	int ret=spi_xfer(spi_fd,headerBuffer,buf,headerLength+readLength);
	if(ret<0) {
		FATAL("ERROR: SPI xfer failed with error code 0x08x\n",ret);
	}
	ret-=headerLength;
	memcpy(readBuffer,buf+headerLength,ret);
#ifdef DEBUG_SPI_PRINT
	PrintBuffer("readfromspi read:",readBuffer,readLength);
#endif//DEBUG_SPI_PRINT
	return 0;
}
