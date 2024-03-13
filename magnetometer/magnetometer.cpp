#include "magnetometer.h"

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<chrono>
#include<thread>
#include<atomic>
#include<assert.h>
#include<algorithm>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"

#if ENABLE_LIBGPIOD

#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <gpiod.h>

#include "platform/gpio.h"
#include "platform/i2c.h"

#include "bmm350_defs.h"

int SetupTest(MagnetometerOutputReadyCallback cb,int freq);

void ReadMagnetometer(MagnetometerOutputReadyCallback cb,int freq) {
	SetupTest(cb,freq);
}

class Magnetometer {
	public:
		Magnetometer(int file) {
			m_file=file;
		}
		//bool ValidateId();
		bool Begin(int freq);
		void Read(V3* mag,float* temp);
		//void ReadRegisters(uint8_t subAddress,uint8_t count,uint8_t* dest);
		//void WriteRegister(uint8_t subAddress,uint8_t data);
	protected:
		int m_file=-1;
		bool isCorrectId1();
		struct bmm350_dev m_dev={0};
};

void Magnetometer::Read(V3* mag,float* temp) {

	//uprintf("\nCompensated Magnetometer and temperature data read with delay\n");

	//uprintf("Timestamp(ms),Mag_X(uT),Mag_Y(uT),Mag_Z(uT),Temperature(degC)\n");

	//rslt=bmm350_delay_us(36000,&m_dev);
	//bmm350_error_codes_print_result("bmm350_delay_us",rslt);

	struct bmm350_mag_temp_data mag_temp_data;
	int rslt=bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data,&m_dev);
	bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data",rslt);
	*mag=V3(mag_temp_data.x,mag_temp_data.y,mag_temp_data.z);
	*temp=mag_temp_data.temperature;
/*
	uprintf("%lu,%f,%f,%f,%f\n",
			(long unsigned int)(coines_get_millis() - time_ms),
			mag_temp_data.x,
			mag_temp_data.y,
			mag_temp_data.z,
			mag_temp_data.temperature);

*/
}

bool Magnetometer::Begin(int freq){
	int rslt=bmm350_interface_init(&m_dev);
	bmm350_error_codes_print_result("bmm350_interface_selection",rslt);

	uint8_t int_ctrl=0;
	
	//struct bmm350_mag_temp_data mag_temp_data;
	
	// Update device structure
	rslt=bmm350_interface_init(&m_dev);
	bmm350_error_codes_print_result("bmm350_interface_selection",rslt);

	// Initialize BMM350
	rslt=bmm350_init(&m_dev);
	bmm350_error_codes_print_result("bmm350_init",rslt);
	uprintf("Read : 0x00 : BMM350 Chip ID : 0x%X\n",m_dev.chip_id);

	// Check PMU busy
	//rslt=bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0,&m_dev);
	//bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0",rslt);

	//uprintf("Expected : 0x07 : PMU cmd busy : 0x0\n");
	//uprintf("Read : 0x07 : PMU cmd busy : 0x%X\n",pmu_cmd_stat_0.pmu_cmd_busy);

	// Get error data
	//rslt=bmm350_get_regs(BMM350_REG_ERR_REG,&err_reg_data,1,&m_dev);
	//bmm350_error_codes_print_result("bmm350_get_error_reg_data",rslt);

	//uprintf("Expected : 0x02 : Error Register : 0x0\n");
	//uprintf("Read : 0x02 : Error Register : 0x%X\n",err_reg_data);

	// Configure interrupt settings
	rslt=bmm350_configure_interrupt(BMM350_PULSED,
										BMM350_ACTIVE_HIGH,
										BMM350_INTR_PUSH_PULL,
										BMM350_MAP_TO_PIN,//BMM350_UNMAP_FROM_PIN,
										&m_dev);
	bmm350_error_codes_print_result("bmm350_configure_interrupt",rslt);

	// Enable data ready interrupt
	rslt=bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT,&m_dev);
	bmm350_error_codes_print_result("bmm350_enable_interrupt",rslt);

	// Get interrupt settings
	rslt=bmm350_get_regs(BMM350_REG_INT_CTRL,&int_ctrl,1,&m_dev);
	bmm350_error_codes_print_result("bmm350_get_regs",rslt);

	//uint8_t set_int_ctrl=(BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | (BMM350_ENABLE << 7);
	//uprintf("Expected : 0x2E : Interrupt control : 0x%X\n",set_int_ctrl);
	//uprintf("Read : 0x2E : Interrupt control : 0x%X\n",int_ctrl);

	if(int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)	{
		uprintf("Data ready enabled\r\n");
	}
	enum bmm350_data_rates odr=BMM350_DATA_RATE_25HZ;

	// Set ODR and performance
	switch(freq) {
		case 200: {
			odr=BMM350_DATA_RATE_200HZ;
			break;
		}
		case 100: {
			odr=BMM350_DATA_RATE_100HZ;
			break;
		}
		case 50: {
			odr=BMM350_DATA_RATE_50HZ;
			break;
		}
		case 25:
		default: {
			odr=BMM350_DATA_RATE_25HZ;
			break;
		}
	}

	rslt=bmm350_set_odr_performance(odr,BMM350_AVERAGING_8,&m_dev);
	bmm350_error_codes_print_result("bmm350_set_odr_performance",rslt);

	// Enable all axis
	rslt=bmm350_enable_axes(BMM350_X_EN,BMM350_Y_EN,BMM350_Z_EN,&m_dev);
	bmm350_error_codes_print_result("bmm350_enable_axes",rslt);

	rslt=bmm350_set_powermode(BMM350_NORMAL_MODE,&m_dev);
	bmm350_error_codes_print_result("bmm350_set_powermode",rslt);

	return rslt == BMM350_OK;
}
//static const uint8_t MAGNETOMETER_CHIP_ID_ADDR=0x00;

static void delayLORT(unsigned int time_ms) {
	usleep(time_ms*1000);
}


bool Magnetometer::isCorrectId1() {
/*
	for(int i=0;i!=0x80;i++) {
		BeginDeviceI2C(m_file,i);

		char reg[1]={BMM350_CHIP_ID_ADDR};
		write(m_file,reg,1);
		uint8_t readReg=0;
		size_t sz=read(m_file,&readReg,1);
		uprintf("dmm350 test address 0x%x  %d read %d\n",i,sz,readReg);
		delay(200);
	}
	uint8_t readReg=0;
	ReadRegisters(BMM350_CHIP_ID_ADDR,1,&readReg);
	return (BMM350_GET_BITS(readReg,BMM350_CHIP_ID)==BMM350_CHIP_ID) ? true : false;
*/
	return false;
}

/*
bool Magnetometer::ValidateId() {
	return isCorrectId1();
	uint8_t readReg=0;
	ReadRegisters(MAGNETOMETER_CHIP_ID_ADDR,1,&readReg);
	uint8_t id=GET_FIELD(BMM350_CHIP_ID,readReg);
	if(id!=BMM350_CHIP_ID) {
		FATAL("BMM350_CHIP_ID %d!=%d\n",BMM350_CHIP_ID,id);
	}
	uprintf("Magnetometer id %d\n",id);

	return true;
}
*/

int g_i2cFile=-1;

int SetupTest(MagnetometerOutputReadyCallback cb,int freq) {

	int file=InitializeI2C("BMM350");

	g_i2cFile=file;

	ConfigGPIO cfg;
	InitGPIO(&cfg);

	Magnetometer magnetometer(file);
	if(!magnetometer.Begin(freq)) {
		FATAL("Magnetometer::Begin failed");
	}
	//int count=0;

	//uint64_t t0=GetTimeEpochMicroseconds();
	uprintf("Magnetometer enter data loop\n");
	while(true) {
		WaitForDataGPIO(&cfg,25);
		//uint64_t t2=GetTimeEpochMicroseconds();
		//if(t2-t0>1000000) {
		//	uprintf("data %d!\n",count);
		//	count=0;
		//	t0=t2;
		//}else{
		//	count++;
		//}
		V3 mag=V3(0,0,0);
		float temp;
		magnetometer.Read(&mag,&temp);
		uint64_t t1=GetTimeEpochMicroseconds();
		cb(mag,temp,t1);
	}
	g_i2cFile=0;

	CloseI2C(file);
	return 0;
}

#else
void ReadMagnetometer(MagnetometerOutputReadyCallback cb,int freq) { }
#endif

