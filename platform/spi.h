#include "shared/types.h"

#ifdef PLATFORM_RPI

extern int spi_fd;

typedef struct {
    uint8_t mode;
    uint8_t bits_per_word;
    uint32_t speed;
    uint16_t delay;
} spi_config_t;

int spi_open(const char *device,spi_config_t config);
int spi_close(int fd);
int spi_xfer(int fd,uint8_t *tx_buffer,uint8_t *rx_buffer,uint8_t len);
int spi_read(int fd,uint8_t *rx_buffer,uint8_t rx_len);
int spi_write(int fd,const uint8_t *tx_buffer,uint8_t tx_len);
void rpi_spi_init();

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer);
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer);

void EnableDebugPrintSPI();
void DisableDebugPrintSPI();

#endif
