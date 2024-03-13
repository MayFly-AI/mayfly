/**
 * Copyright (C) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    coines.h
 * @brief   This file contains COINES layer function prototypes, variable declarations and Macro definitions
 *
 */

/*!
 * @addtogroup coines_api
 * @{*/

#ifndef COINES_H_
#define COINES_H_


/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
/*! coines success code */
#define COINES_SUCCESS                  0
/*! coines error code - failure */
#define COINES_E_FAILURE               -1
#define COINES_E_NULL_PTR              -9

struct coines_board_info
{
    uint16_t hardware_id; /*< Board hardware ID */
    uint16_t software_id; /*< Board software ID */
    uint8_t board; /*< Type of the board like APP2.0, Arduino Due */
    uint16_t shuttle_id; /*< Shuttle ID of the sensor connected */
};
enum coines_i2c_mode
{
    COINES_I2C_STANDARD_MODE = 0x00, /*< I2C speed in standard mode */
    COINES_I2C_FAST_MODE = 0x01, /*< I2C speed in fast mode */
    COINES_I2C_SPEED_3_4_MHZ = 0x02, /*< I2C speed in 3.4 MHz */
    COINES_I2C_SPEED_1_7_MHZ = 0x03 /*< I2C speed in 1.7 MHz */
};

enum coines_comm_intf
{
    COINES_COMM_INTF_USB, /*< communication interface USB */
    COINES_COMM_INTF_VCOM, /*< communication interface VCOM */
    COINES_COMM_INTF_BLE /*< communication interface BLE */
};


enum coines_i2c_bus
{
    COINES_I2C_BUS_0, /*< I2C bus 0 */
    COINES_I2C_BUS_1 /*< I2C bus 1 */
};

int16_t coines_get_board_info(struct coines_board_info *data);

int16_t coines_open_comm_intf();
void coines_soft_reset();

uint32_t coines_get_millis();
int8_t coines_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);
int8_t coines_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);
void coines_delay_msec(uint32_t delay_ms);
void coines_delay_usec(uint32_t delay_us);
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode);
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt);
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type);


#ifdef __cplusplus
}
#endif

#endif /* COINES_H_ */

/** @}*/
