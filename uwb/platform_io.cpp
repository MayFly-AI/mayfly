
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"

#include "platform/gpio.h"
#include "platform_io.h"

//void gpio_init() {}

// Configures the interrupt. Select the right respective I/O pin and disables it.
// MAX: Should be set up with callback function on interrupt?
void dw_irq_init(uint8_t IRQ_pin) {
	//uprintf("dw_irq_init\n");
	RPI_GPIO::setup_gpio(IRQ_pin,RPI_GPIO::PIN_MODE::INPUT,0);
}

// reset_DW IC
// DW_RESET pin on DW IC has 2 functions
// In general it is output, but it also can be used to reset the digital
// part of DW IC by driving this pin low.
// Note, the DW_RESET pin should not be driven high externally.
void reset_DWIC(uint8_t RST_pin) {
	//uprintf("reset_DWIC\n");
	RPI_GPIO::setup_gpio(RST_pin,RPI_GPIO::PIN_MODE::OUTPUT,0);
	RPI_GPIO::output_gpio(RST_pin,0);
	usleep(2000);
	RPI_GPIO::setup_gpio(RST_pin,RPI_GPIO::PIN_MODE::INPUT,0);
	usleep(10000);
}

void UART_puts(const char* s){
	uprintf("UART_puts %s\n",(const char*)s);
}

void wrap_port_set_dw_ic_spi_fastrate() {
	//uprintf("port_set_dw_ic_spi_fastrate. Not set!\n");
}

// Reset and initialize DW chip.
void wrap_reset_DWIC() {
	//uprintf("reset allready done from main\n");
}
void wrap_Sleep(int ms) {
	usleep(ms*1000);
}
// Wrapper function to be used by decadriver. Declared in deca_device_api.h
void deca_sleep(unsigned int time_ms) {
	usleep(time_ms*1000);
}
// Wrapper function to be used by decadriver. Declared in deca_device_api.h
void deca_usleep(unsigned long time_us) {
	usleep(time_us);
}

