#ifndef _PLATFORM_IO_H_
#define _PLATFORM_IO_H_

#include <cstdio>

#ifdef __cplusplus
extern "C"
{
#endif

// SYS_STATE_LO register errors
#define DW_SYS_STATE_TXERR          0xD0000         // TSE is in TX but TX is in IDLE in SYS_STATE_LO register

void wrap_port_set_dw_ic_spi_fastrate();
void wrap_reset_DWIC();
void wrap_Sleep(int ms);
void deca_sleep(unsigned int time_ms);
void deca_usleep(unsigned long time_us);

//void EnableDebugPrintSPI();
//void DisableDebugPrintSPI();
//void PrintBuffer(const char* name,const uint8_t *dataBuffer,uint16_t dataLength);

void UART_puts(const char* s);


void gpio_init();
void dw_irq_init(uint8_t IRQ_pin);
void reset_DWIC(uint8_t RST_pin);

#ifdef __cplusplus
}
#endif

#endif//_PLATFORM_IO_H_
