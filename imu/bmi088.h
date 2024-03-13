/*/*
* Based on Brian R Taylor Bolder Flight Systems
*/

#pragma once

// constants
static const uint8_t ACC_CHIP_ID=0x1E;
static const uint8_t ACC_RESET_CMD=0xB6;
static const uint8_t ACC_ENABLE_CMD=0x04;
static const uint8_t ACC_DISABLE_CMD=0x00;
static const uint8_t ACC_SUSPEND_MODE_CMD=0x03;
static const uint8_t ACC_ACTIVE_MODE_CMD=0x00;
static const uint8_t ACC_INT_INPUT=0x11;
static const uint8_t ACC_INT_OUTPUT=0x08;
static const uint8_t ACC_INT_OPENDRAIN=0x04;
static const uint8_t ACC_INT_PUSHPULL=0x00;
static const uint8_t ACC_INT_LVL_HIGH=0x02;
static const uint8_t ACC_INT_LVL_LOW=0x00;
static const uint8_t ACC_POS_SELF_TEST=0x0D;
static const uint8_t ACC_NEG_SELF_TEST=0x09;
static const uint8_t ACC_DIS_SELF_TEST=0x00;
// registers

static const uint8_t ACC_CHIP_ID_ADDR=0x00;
static const uint8_t ACC_CHIP_ID_MASK=0xFF;
static const uint8_t ACC_CHIP_ID_POS=0;
static const uint8_t ACC_FATAL_ERR_ADDR=0x02;
static const uint8_t ACC_FATAL_ERR_MASK=0x01;
static const uint8_t ACC_FATAL_ERR_POS=0;
static const uint8_t ACC_ERR_CODE_ADDR=0x02;
static const uint8_t ACC_ERR_CODE_MASK=0x1C;
static const uint8_t ACC_ERR_CODE_POS=2;
static const uint8_t ACC_DRDY_ADDR=0x03;
static const uint8_t ACC_DRDY_MASK=0x80;
static const uint8_t ACC_DRDY_POS=7;
static const uint8_t ACC_ODR_ADDR=0x40;
static const uint8_t ACC_ODR_MASK=0xFF;
static const uint8_t ACC_ODR_POS=0;
static const uint8_t ACC_RANGE_ADDR=0x41;
static const uint8_t ACC_RANGE_MASK=0x03;
static const uint8_t ACC_RANGE_POS=0;
static const uint8_t ACC_INT1_IO_CTRL_ADDR=0x53;
static const uint8_t ACC_INT1_IO_CTRL_MASK=0x1F;
static const uint8_t ACC_INT1_IO_CTRL_POS=0;
static const uint8_t ACC_INT2_IO_CTRL_ADDR=0x54;
static const uint8_t ACC_INT2_IO_CTRL_MASK=0x1F;
static const uint8_t ACC_INT2_IO_CTRL_POS=0;
static const uint8_t ACC_INT1_DRDY_ADDR=0x58;
static const uint8_t ACC_INT1_DRDY_MASK=0x04;
static const uint8_t ACC_INT1_DRDY_POS=2;
static const uint8_t ACC_INT2_DRDY_ADDR=0x58;
static const uint8_t ACC_INT2_DRDY_MASK=0x40;
static const uint8_t ACC_INT2_DRDY_POS=6;
static const uint8_t ACC_SELF_TEST_ADDR=0x6D;
static const uint8_t ACC_SELF_TEST_MASK=0xFF;
static const uint8_t ACC_SELF_TEST_POS=0;
static const uint8_t ACC_PWR_CONF_ADDR=0x7C;
static const uint8_t ACC_PWR_CONF_MASK=0xFF;
static const uint8_t ACC_PWR_CONF_POS=0;
static const uint8_t ACC_PWR_CNTRL_ADDR=0x7D;
static const uint8_t ACC_PWR_CNTRL_MASK=0xFF;
static const uint8_t ACC_PWR_CNTRL_POS=0;
static const uint8_t ACC_SOFT_RESET_ADDR=0x7E;
static const uint8_t ACC_SOFT_RESET_MASK=0xFF;
static const uint8_t ACC_SOFT_RESET_POS=0;
static const uint8_t ACC_ACCEL_DATA_ADDR=0x12;
static const uint8_t ACC_TEMP_DATA_ADDR=0x22;

// constants
static const uint8_t GYRO_CHIP_ID=0x0F;
static const uint8_t GYRO_RESET_CMD=0xB6;
static const uint8_t GYRO_ENABLE_DRDY_INT=0x80;
static const uint8_t GYRO_DIS_DRDY_INT=0x00;
static const uint8_t GYRO_INT_OPENDRAIN=0x02;
static const uint8_t GYRO_INT_PUSHPULL=0x00;
static const uint8_t GYRO_INT_LVL_HIGH=0x01;
static const uint8_t GYRO_INT_LVL_LOW=0x00;
// registers
static const uint8_t GYRO_CHIP_ID_ADDR=0x00;
static const uint8_t GYRO_CHIP_ID_MASK=0xFF;
static const uint8_t GYRO_CHIP_ID_POS=0;
static const uint8_t GYRO_DRDY_ADDR=0x0A;
static const uint8_t GYRO_DRDY_MASK=0x80;
static const uint8_t GYRO_DRDY_POS=7;
static const uint8_t GYRO_RANGE_ADDR=0x0F;
static const uint8_t GYRO_RANGE_MASK=0xFF;
static const uint8_t GYRO_RANGE_POS=0;
static const uint8_t GYRO_ODR_ADDR=0x10;
static const uint8_t GYRO_ODR_MASK=0xFF;
static const uint8_t GYRO_ODR_POS=0;
static const uint8_t GYRO_SOFT_RESET_ADDR=0x14;
static const uint8_t GYRO_SOFT_RESET_MASK=0xFF;
static const uint8_t GYRO_SOFT_RESET_POS=0;
static const uint8_t GYRO_INT_CNTRL_ADDR=0x15;
static const uint8_t GYRO_INT_CNTRL_MASK=0xFF;
static const uint8_t GYRO_INT_CNTRL_POS=0;
static const uint8_t GYRO_INT3_IO_CTRL_ADDR=0x16;
static const uint8_t GYRO_INT3_IO_CTRL_MASK=0x03;
static const uint8_t GYRO_INT3_IO_CTRL_POS=0;
static const uint8_t GYRO_INT4_IO_CTRL_ADDR=0x16;
static const uint8_t GYRO_INT4_IO_CTRL_MASK=0x0C;
static const uint8_t GYRO_INT4_IO_CTRL_POS=2;
static const uint8_t GYRO_INT3_DRDY_ADDR=0x18;
static const uint8_t GYRO_INT3_DRDY_MASK=0x01;
static const uint8_t GYRO_INT3_DRDY_POS=0;
static const uint8_t GYRO_INT4_DRDY_ADDR=0x18;
static const uint8_t GYRO_INT4_DRDY_MASK=0x80;
static const uint8_t GYRO_INT4_DRDY_POS=7;
static const uint8_t GYRO_DATA_ADDR=0x02;

#define ACC_ODR_1600HZ 0x0c
#define ACC_ODR_800HZ 0x0b
#define ACC_ODR_400HZ 0x0a
#define ACC_ODR_200HZ 0x09
#define ACC_ODR_100HZ 0x08
#define ACC_ODR_50HZ 0x07
#define ACC_ODR_25HZ 0x06
#define ACC_ODR_12HZ 0x05

#define ACC_OCR4 (0x00<<4)
#define ACC_OCR2 (0x01<<4)
#define ACC_NORMAL (0x02<<4)

#define ACC_RANGE_3G 0x00
#define ACC_RANGE_6G 0x01
#define ACC_RANGE_12G 0x02
#define ACC_RANGE_24G 0x03

class Accel {
	public:
		Accel(int file) {
			m_file=file;
		}
		void WriteRegister(uint8_t subAddress,uint8_t data,bool wait=false);
		void ReadRegisters(uint8_t subAddress,uint8_t count,uint8_t* dest);
		bool isCorrectId1();
		bool SetMode(bool active);
		bool SetPower(bool enable);
		void SoftReset();
		bool SetOdr(uint8_t odr,uint8_t bwp);
		bool SetRange(uint8_t range);
		bool PinModeInt1(uint8_t pin_io,uint8_t pin_mode,uint8_t active_lvl);
		bool PinModeInt2(uint8_t pin_io,uint8_t pin_mode,uint8_t active_lvl);
		bool MapDrdyInt1(bool enable);
		bool MapDrdyInt2(bool enable);
		bool Read(V3* acc,uint32_t* timeCounter,float* temperature);
		bool Begin();
	protected:
		int m_file=-1;
		uint32_t m_prevTimeCounter=0;
		float m_accelRangeMSS=0;
		bool SelfTest();
};

#define GYRO_RANGE_2000DPS 0x00
#define GYRO_RANGE_1000DPS 0x01 
#define GYRO_RANGE_500DPS 0x02
#define GYRO_RANGE_250DPS 0x03
#define GYRO_RANGE_125DPS 0x04

#define GYRO_ODR_2000HZ_BW_532HZ 0x80
#define GYRO_ODR_2000HZ_BW_230HZ 0x81
#define GYRO_ODR_1000HZ_BW_116HZ 0x82
#define GYRO_ODR_400HZ_BW_47HZ 0x83
#define GYRO_ODR_200HZ_BW_23HZ 0x84
#define GYRO_ODR_100HZ_BW_12HZ 0x85
#define GYRO_ODR_200HZ_BW_64HZ 0x86
#define GYRO_ODR_100HZ_BW_32HZ 0x87

class Gyro {
	public:
		Gyro(int file) {
			m_file=file;
		}
		void ReadRegisters(uint8_t subAddress,uint8_t count,uint8_t* dest);
		void WriteRegister(uint8_t subAddress,uint8_t data,bool wait=false);
		void ValidateId();
		bool SetDataInterupt(bool enable);
		bool MapDrdyInt3(bool enable);
		bool MapDrdyInt4(bool enable);
		bool PinModeInt3(uint8_t pin_mode,uint8_t active_lvl);
		bool PinModeInt4(uint8_t pin_mode,uint8_t active_lvl);
		bool Begin();
		bool SetOdr(uint8_t odr);
		void Read(V3* rads);
		void SoftReset();
		bool SetRange(uint8_t range);
	protected:
		float m_gyroRangeRads=0;
		int m_file=-1;
};
