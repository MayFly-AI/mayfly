#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "shared/misc.h"
#include "platform/i2c.h"

#include "coines.h"


extern int g_i2cFile;

extern "C" {

#define COINES_DATA_BUF_SIZE              (1024)

typedef enum
{
    COINES_BOARD_DD = 0xC0
    /*< DD */
} coines_board_t;
typedef struct
{
    uint8_t buffer[COINES_DATA_BUF_SIZE]; /*< Data buffer */
    uint32_t buffer_size; /*< buffer size */
    uint8_t error; /*< error code */
    coines_board_t board_type; /*< board type */
} coines_command_t;

#define  COINES_CMD_ID                                     UINT8_C(0xAA)

coines_command_t comm_buf;
void comm_intf_init_command_header(uint8_t cmd_type, uint8_t int_feature)
{
    comm_buf.buffer[0] = COINES_CMD_ID;
    comm_buf.buffer[1] = 0;
    comm_buf.buffer[2] = cmd_type;
    comm_buf.buffer[3] = int_feature;
    comm_buf.buffer_size = 4;
}

int16_t coines_get_board_info(struct coines_board_info *data) {
	return COINES_SUCCESS;
}

int16_t coines_open_comm_intf() {
	//g_file=InitializeI2C("COINES");
	return COINES_SUCCESS;
}
void coines_soft_reset() {
	FATAL("coines_soft_reset");
}

uint32_t coines_get_millis() {
	static uint64_t st=GetTimeEpochMicroseconds();
	return (uint32_t)(GetTimeEpochMicroseconds()-st);
}

int8_t coines_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
	LockI2C();
	BeginDeviceI2C(g_i2cFile,dev_addr);
	int ret=ReadFromAddressI2C(g_i2cFile,reg_data,reg_addr,count);
	UnlockI2C();
	if(ret==-1) {
		FATAL("unable to read from address");
	}
	return COINES_SUCCESS;
//	if(ioctl(g_file,I2C_SLAVE,dev_addr)<0) {
//		FATAL("Failed to acquire bus access and/or talk to slave.\n");
//	}
//	char reg[1]={reg_addr};
//	write(g_file,reg,1);
//	size_t sz=read(g_file,reg_data,count);
//	return sz==count ? BMM350_INTF_RET_SUCCESS:-1;
}

int8_t coines_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
	if(count!=1)
		FATAL("coines_write_i2c count %d",count);

	LockI2C();
	BeginDeviceI2C(g_i2cFile,dev_addr);
	WriteToAddressI2C(g_i2cFile,reg_addr,*reg_data);
	UnlockI2C();
	return COINES_SUCCESS;
/*
	if(ioctl(g_file,I2C_SLAVE,dev_addr)<0) {
		FATAL("Failed to acquire bus access and/or talk to slave.\n");
	}
	if(count!=1)
		FATAL("coines_write_i2c count %d",count);
		
	char config[2];
	config[0]=reg_addr;
	config[1]=*reg_data;
	size_t sz=write(g_file,config,2);
	if((int)sz==count+1)
		return BMM350_INTF_RET_SUCCESS;
	return -1;
*/
}
void coines_delay_msec(uint32_t delay_ms) {
	usleep(delay_ms*1000);
	//FATAL("coines_delay_msec");
}
void coines_delay_usec(uint32_t delay_us) {
	usleep(delay_us);
	//FATAL("coines_delay_usec");
}
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode) {
	//FATAL("coines_config_i2c_bus");
	return 0;
}
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt) {
	//FATAL("coines_set_shuttleboard_vdd_vddio_config");
	return 0;
}
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type) {
	//FATAL("coines_close_comm_intf");
	return 0;
}

}; //extern C
