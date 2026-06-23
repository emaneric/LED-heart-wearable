#include "main.h"

uint8_t LIS2DU12_init(SPI_HandleTypeDef *hspi);
uint8_t LIS2DU12_read_FIFO(SPI_HandleTypeDef *hspi, uint8_t *FIFO_length, int16_t acceleration_data[256][3]);