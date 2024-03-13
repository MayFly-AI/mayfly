#include <unistd.h>

#include "shared/file.h"
#include "shared/std_ext.h"

#include "shared/types.h"
#include "shared/misc.h"


#include "magnetometer.h"
#include "platform/i2c.h"
#include "platform/gpio.h"

int main3() {
	coines_open_comm_intf();
	usleep(BMM350_START_UP_TIME_FROM_POR);
	uint8_t soft_reset=BMM350_CMD_SOFTRESET;
	coines_write_i2c(BMM350_I2C_ADSEL_SET_LOW,BMM350_REG_CMD,&soft_reset,1);
	usleep(BMM350_SOFT_RESET_DELAY);

	int len=1;
    uint16_t temp_len=len+BMM350_DUMMY_BYTES;
    uint8_t temp_buf[BMM350_READ_BUFFER_LENGTH];
	coines_read_i2c(BMM350_I2C_ADSEL_SET_LOW,BMM350_REG_CHIP_ID,temp_buf,temp_len);
	uint8_t chip_id=temp_buf[BMM350_DUMMY_BYTES];
	uprintf("chip_id %d\n",chip_id);

    struct bmm350_dev dev={ 0 };
    uint8_t loop=20;
    int rslt=bmm350_interface_init(&dev);


    // Configure interrupt settings
    rslt = bmm350_configure_interrupt(BMM350_PULSED,
                                      BMM350_ACTIVE_HIGH,
                                      BMM350_INTR_PUSH_PULL,
                                      BMM350_UNMAP_FROM_PIN,
                                      &dev);
	rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev);
    bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

	uint8_t int_ctrl = 0;
	uint8_t set_int_ctrl;
	uint32_t time_ms = 0;

	// Get interrupt settings
	rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, &dev);
	bmm350_error_codes_print_result("bmm350_get_regs", rslt);

	set_int_ctrl = (BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | (BMM350_ENABLE << 7);

	uprintf("Expected : 0x2E : Interrupt control : 0x%X\n", set_int_ctrl);
	uprintf("Read : 0x2E : Interrupt control : 0x%X\n", int_ctrl);

	if (int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)
	{
		uprintf("Data ready enabled\r\n");
	}

	// Set ODR and performance
	rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &dev);
	bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

	// Enable all axis
	rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
	bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

	rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
	bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

	loop = 20;

	uprintf("\nCompensated Magnetometer and temperature data read with delay\n");

	uprintf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

	// Time in milliseconds
	time_ms = coines_get_millis();

	while (loop)
	{
		rslt = bmm350_delay_us(36000, &dev);
		bmm350_error_codes_print_result("bmm350_delay_us", rslt);

	    struct bmm350_mag_temp_data mag_temp_data;
		rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
		bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

		uprintf("%lu, %f, %f, %f, %f\n",
				(long unsigned int)(coines_get_millis() - time_ms),
				mag_temp_data.x,
				mag_temp_data.y,
				mag_temp_data.z,
				mag_temp_data.temperature);

		loop--;
	}


	return 0;
}


// This function starts the execution of program
int main1(void)
{
    // Status of api are returned to this variable
    int8_t rslt;

    // Sensor initialization configuration
    struct bmm350_dev dev={ 0 };

    uint8_t loop=20;
    uint32_t secs,nano_secs=0;

    // Update device structure
    rslt=bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_selection",rslt);

    // Initialize BMM350
    rslt=bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init",rslt);

    uprintf("Read : 0x00 : BMM350 Chip ID : 0x%X\n",dev.chip_id);

    // Set ODR and performance
    rslt=bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ,BMM350_AVERAGING_4,&dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance",rslt);

    // Enable all axis
    rslt=bmm350_enable_axes(BMM350_X_EN,BMM350_Y_EN,BMM350_Z_EN,&dev);
    bmm350_error_codes_print_result("bmm350_enable_axes",rslt);

    if (rslt == BMM350_OK)
    {
        rslt=bmm350_set_powermode(BMM350_SUSPEND_MODE,&dev);
        bmm350_error_codes_print_result("bmm350_set_powermode",rslt);

        uprintf("\nSensortime in suspend mode\n");
        uprintf("Time(secs)\n");

        while (loop > 0)
        {
            rslt=bmm350_read_sensortime(&secs,&nano_secs,&dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime",rslt);

            uprintf("%lu.%09lu\n",(long unsigned int)secs,(long unsigned int)nano_secs);

            loop--;
        }

        rslt=bmm350_set_ctrl_user(BMM350_CFG_SENS_TIM_AON_EN,&dev);
        bmm350_error_codes_print_result("bmm350_set_ctrl_user",rslt);

        loop=20;

        uprintf("\nSensortime in forced mode\n");

        uprintf("Time(secs)\n");

        while (loop > 0)
        {
            rslt=bmm350_set_powermode(BMM350_FORCED_MODE,&dev);
            bmm350_error_codes_print_result("bmm350_set_powermode",rslt);

            rslt=bmm350_delay_us(40000,&dev);
            bmm350_error_codes_print_result("bmm350_delay_us",rslt);

            rslt=bmm350_read_sensortime(&secs,&nano_secs,&dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime",rslt);

            uprintf("\n%lu.%09lu\n",(long unsigned int)secs,(long unsigned int)nano_secs);

            rslt=bmm350_read_sensortime(&secs,&nano_secs,&dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime",rslt);

            uprintf("%lu.%09lu\n",(long unsigned int)secs,(long unsigned int)nano_secs);

            rslt=bmm350_read_sensortime(&secs,&nano_secs,&dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime",rslt);

            uprintf("%lu.%09lu\n",(long unsigned int)secs,(long unsigned int)nano_secs);

            loop--;
        }

        rslt=bmm350_set_powermode(BMM350_NORMAL_MODE,&dev);
        bmm350_error_codes_print_result("bmm350_set_powermode",rslt);

        // Set ODR and performance
        rslt=bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ,BMM350_AVERAGING_2,&dev);
        bmm350_error_codes_print_result("bmm350_set_odr_performance",rslt);

        loop=20;

        uprintf("\nChange in ODR\n");

        uprintf("Time(secs)\n");

        while (loop > 0)
        {
            rslt=bmm350_delay_us(11000,&dev);
            bmm350_error_codes_print_result("bmm350_delay_us",rslt);

            rslt=bmm350_read_sensortime(&secs,&nano_secs,&dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime",rslt);

            uprintf("%lu.%09lu\n",(long unsigned int)secs,(long unsigned int)nano_secs);

            loop--;
        }
    }

    bmm350_coines_deinit();

    return rslt;
}

int main5(void);
int main7(void);

int main(int argc,char *argv[]) {

	InitializeI2C("BMM350 EXAMPLE");
	ConfigGPIO cfg;
	InitGPIO(&cfg);
	//int count=0;

	//main5();
	int count=0;
	auto cb=[&](const V3& mag,float temp,const uint64_t& t) {
		uint32_t time=(int)(GetTimeMicroseconds()/(1000000L/256L));
		uprintf("count %d sec %.02f mag %f %f %f time %llu\n",count,(float)time/256.0f,mag[0],mag[1],mag[2],t);
		count++;
	};
	ReadMagnetometer(cb,25);
	return 0;
}




// This function starts the execution of program
int main5(void)
{
	// Status of api are returned to this variable
	int8_t rslt;

	// Sensor initialization configuration
	struct bmm350_dev dev = { 0 };

	uint8_t int_status, int_ctrl=0;
	uint8_t err_reg_data=0;
	uint8_t loop, set_int_ctrl;
	uint32_t time_ms = 0;

	struct bmm350_mag_temp_data mag_temp_data;
	struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

	// Update device structure
	rslt = bmm350_interface_init(&dev);
	bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

	// Initialize BMM350
	rslt = bmm350_init(&dev);
	bmm350_error_codes_print_result("bmm350_init", rslt);
	uprintf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

	// Check PMU busy
	rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
	bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

	uprintf("Expected : 0x07 : PMU cmd busy : 0x0\n");
	uprintf("Read : 0x07 : PMU cmd busy : 0x%X\n", pmu_cmd_stat_0.pmu_cmd_busy);

	// Get error data
	rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, &dev);
	bmm350_error_codes_print_result("bmm350_get_error_reg_data", rslt);

	uprintf("Expected : 0x02 : Error Register : 0x0\n");
	uprintf("Read : 0x02 : Error Register : 0x%X\n", err_reg_data);

	// Configure interrupt settings
	rslt = bmm350_configure_interrupt(BMM350_PULSED,
										BMM350_ACTIVE_HIGH,
										BMM350_INTR_PUSH_PULL,
										BMM350_MAP_TO_PIN,
										&dev);
	bmm350_error_codes_print_result("bmm350_configure_interrupt", rslt);

	// Enable data ready interrupt
	rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev);
	bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

	// Get interrupt settings
	rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, &dev);
	bmm350_error_codes_print_result("bmm350_get_regs", rslt);

	set_int_ctrl = (BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | (BMM350_ENABLE << 7);

	uprintf("Expected : 0x2E : Interrupt control : 0x%X\n", set_int_ctrl);
	uprintf("Read : 0x2E : Interrupt control : 0x%X\n", int_ctrl);

	if (int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)
	{
		uprintf("Data ready enabled\r\n");
	}

	// Set ODR and performance
	rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &dev);
	bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

	// Enable all axis
	rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
	bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

	if (rslt == BMM350_OK)
	{
		rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
		bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

		loop = 20;

		uprintf("\nCompensated Magnetometer and temperature data read with delay\n");

		uprintf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

		// Time in milliseconds
		time_ms = coines_get_millis();

		while (loop)
		{
			rslt = bmm350_delay_us(36000, &dev);
			bmm350_error_codes_print_result("bmm350_delay_us", rslt);

			rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
			bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

			uprintf("%lu, %f, %f, %f, %f\n",
					(long unsigned int)(coines_get_millis() - time_ms),
					mag_temp_data.x,
					mag_temp_data.y,
					mag_temp_data.z,
					mag_temp_data.temperature);

			loop--;
		}

		loop = 22;

		uprintf("\nCompensated Magnetometer and temperature data read with INT\n");

		uprintf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

		// Time in milliseconds
		time_ms = coines_get_millis();

		while (loop)
		{
			int_status = 0;

			// Get data ready interrupt status
			rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, &dev);
			bmm350_error_codes_print_result("bmm350_get_regs", rslt);

			// Check if data ready interrupt occurred
			if (int_status & BMM350_DRDY_DATA_REG_MSK)
			{
				rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
				bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

				uprintf("%lu, %f, %f, %f, %f\n",
						(long unsigned int)(coines_get_millis() - time_ms),
						mag_temp_data.x,
						mag_temp_data.y,
						mag_temp_data.z,
						mag_temp_data.temperature);

				loop--;
			}
		}
	}

	//bmm350_coines_deinit();

	return rslt;
}

// This function starts the execution of program
int main7(void)
{
	// Status of api are returned to this variable
	int8_t rslt;

	// Sensor initialization configuration
	struct bmm350_dev dev = { 0 };

	uint8_t int_ctrl=0;
	uint8_t loop, set_int_ctrl;
	uint32_t time_ms = 0;

	struct bmm350_mag_temp_data mag_temp_data;
	
	// Update device structure
	rslt = bmm350_interface_init(&dev);
	bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

	// Initialize BMM350
	rslt = bmm350_init(&dev);
	bmm350_error_codes_print_result("bmm350_init", rslt);
	uprintf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

	// Check PMU busy
	//rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
	//bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

	//uprintf("Expected : 0x07 : PMU cmd busy : 0x0\n");
	//uprintf("Read : 0x07 : PMU cmd busy : 0x%X\n", pmu_cmd_stat_0.pmu_cmd_busy);

	// Get error data
	//rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, &dev);
	//bmm350_error_codes_print_result("bmm350_get_error_reg_data", rslt);

	//uprintf("Expected : 0x02 : Error Register : 0x0\n");
	//uprintf("Read : 0x02 : Error Register : 0x%X\n", err_reg_data);

	// Configure interrupt settings
	rslt = bmm350_configure_interrupt(BMM350_PULSED,
										BMM350_ACTIVE_HIGH,
										BMM350_INTR_PUSH_PULL,
										BMM350_UNMAP_FROM_PIN,
										&dev);
	bmm350_error_codes_print_result("bmm350_configure_interrupt", rslt);

	// Enable data ready interrupt
	rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev);
	bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

	// Get interrupt settings
	rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, &dev);
	bmm350_error_codes_print_result("bmm350_get_regs", rslt);

	set_int_ctrl = (BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | (BMM350_ENABLE << 7);

	uprintf("Expected : 0x2E : Interrupt control : 0x%X\n", set_int_ctrl);
	uprintf("Read : 0x2E : Interrupt control : 0x%X\n", int_ctrl);

	if (int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)
	{
		uprintf("Data ready enabled\r\n");
	}

	// Set ODR and performance
	rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &dev);
	bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

	// Enable all axis
	rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
	bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

	if (rslt == BMM350_OK)
	{
		rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
		bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

		loop = 20;

		uprintf("\nCompensated Magnetometer and temperature data read with delay\n");

		uprintf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

		// Time in milliseconds
		time_ms = coines_get_millis();

		while (loop)
		{
			rslt = bmm350_delay_us(36000, &dev);
			bmm350_error_codes_print_result("bmm350_delay_us", rslt);

			rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
			bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

			uprintf("%lu, %f, %f, %f, %f\n",
					(long unsigned int)(coines_get_millis() - time_ms),
					mag_temp_data.x,
					mag_temp_data.y,
					mag_temp_data.z,
					mag_temp_data.temperature);

			loop--;
		}
/*
		loop = 22;

		uprintf("\nCompensated Magnetometer and temperature data read with INT\n");

		uprintf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

		// Time in milliseconds
		time_ms = coines_get_millis();

		while (loop)
		{
			int_status = 0;

			// Get data ready interrupt status
			rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, &dev);
			bmm350_error_codes_print_result("bmm350_get_regs", rslt);

			// Check if data ready interrupt occurred
			if (int_status & BMM350_DRDY_DATA_REG_MSK)
			{
				rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
				bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

				uprintf("%lu, %f, %f, %f, %f\n",
						(long unsigned int)(coines_get_millis() - time_ms),
						mag_temp_data.x,
						mag_temp_data.y,
						mag_temp_data.z,
						mag_temp_data.temperature);

				loop--;
			}
		}
*/
	}

	//bmm350_coines_deinit();

	return rslt;
}

