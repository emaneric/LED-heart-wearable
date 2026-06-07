#include "LIS2DU12.h"
#include "stm32u0xx_hal_spi.h"

#define SPI_TIMEOUT 1000
#define WHO_AM_I 0x43

uint8_t rx_buffer[128] = {0};
uint8_t tx_buffer[128] = {0};

uint8_t LIS2DU12_init(SPI_HandleTypeDef *hspi){


    tx_buffer[0] = WHO_AM_I | 0x80;

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, SPI_TIMEOUT);
    __NOP();

    if (status == HAL_OK){
        return 0;
    }
    else{
        return 1;
    }
}