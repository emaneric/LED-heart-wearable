#include "main.h"

uint8_t LIS2DW12_init(SPI_HandleTypeDef *hspi);
uint8_t LIS2DW12_read_FIFO(SPI_HandleTypeDef *hspi, uint8_t *FIFO_length, int8_t acceleration_data[256][3]);
uint8_t LIS2DW12_configure_sleep(SPI_HandleTypeDef *hspi);
uint8_t LIS2DW12_clear_interrupt_sources(SPI_HandleTypeDef *hspi);
