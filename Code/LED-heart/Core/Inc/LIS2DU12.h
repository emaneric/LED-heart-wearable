#include "main.h"

uint8_t LIS2DU12_init(SPI_HandleTypeDef *hspi);
uint8_t LIS2DU12_read_FIFO(SPI_HandleTypeDef *hspi, uint8_t *FIFO_length, int8_t acceleration_data[256][3]);
uint8_t LIS2DU12_configure_sleep(SPI_HandleTypeDef *hspi);