#include "LIS2DU12.h"
#include "stm32u0xx_hal.h"
#include "stm32u0xx_hal_gpio.h"
#include "stm32u0xx_hal_spi.h"
#include <stdio.h>

#define SPI_READ_BIT(reg)  ((reg) | 0x80)
#define SPI_WRITE_BIT(reg)  ((reg) & 0x7F)
#define SPI_TIMEOUT 1000

typedef enum {
    LIS2DU12_REG_CTRL2       = 0x11,
    LIS2DU12_REG_CTRL5       = 0x14,
    LIS2DU12_REG_FIFO_CTRL   = 0x15,
    LIS2DU12_REG_FIFO_WTM    = 0x16,
    LIS2DU12_REG_FIFO_STATUS1 = 0x26,
    LIS2DU12_REG_FIFO_STATUS2 = 0x27,
    LIS2DU12_REG_WHO_AM_I    = 0x43,
} LIS2DU12_RegAddr_t;


union {
    uint8_t raw;
    struct {
        uint8_t reserved   : 3;  // bits 0-2, must stay 0
        uint8_t INT1_DRDY  : 1;  // bit 3
        uint8_t INT1_F_OVR : 1;  // bit 4
        uint8_t INT1_F_FTH : 1;  // bit 5
        uint8_t INT1_F_FULL: 1;  // bit 6
        uint8_t INT1_BOOT  : 1;  // bit 7
    } field;
} CTRL2_reg;

union {
    uint8_t raw;
    struct {
        uint8_t FS   : 2;  // bit 0-1
        uint8_t BW  : 2;  // bits 2-3
        uint8_t ODR : 4;  // bits 4-7
    } field;
} CTRL5_reg;

union {
    uint8_t raw;
    struct {
        uint8_t FIFO_MODE   : 3;  // bit 0-2
        uint8_t STOP_ON_FTH  : 1;  // bit 3
        uint8_t reserved : 2;  // bits 4-5
        uint8_t FIFO_DEPTH   : 1;  // bit 6
        uint8_t ROUNDING_XYZ   : 1;  // bit 7
    } field;
} FIFO_CTRL_reg;

union {
    uint8_t raw;
    struct {
        uint8_t FTH   : 7;  // bits 0-6
        uint8_t reserved : 1;  // bit 7
    } field;
} FIFO_WTM_reg;

union {
    uint8_t raw;
    struct {
        uint8_t reserved   : 6;  // bits 0-5
        uint8_t FIFO_OVR : 1;  // bit 6
        uint8_t FTH : 1;  // bit 7
    } field;
} FIFO_STATUS1_reg;

union {
    uint8_t raw;
    struct {
        uint8_t FSS   : 8;  // bits 0-7
    } field;
} FIFO_STATUS2_reg;


HAL_StatusTypeDef LIS2DU12_write_register(SPI_HandleTypeDef *hspi, LIS2DU12_RegAddr_t reg_addr, uint8_t reg_val){

    uint8_t tx_buffer[2] = {0};
    tx_buffer[0] = reg_addr & 0x7F; //set the MSB to 1 to indicate a write
    tx_buffer[1] = reg_val;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, tx_buffer, 2, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    return status;
}

HAL_StatusTypeDef LIS2DU12_read_register(SPI_HandleTypeDef *hspi, LIS2DU12_RegAddr_t reg_addr, uint8_t *reg_val){

    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    tx_buffer[0] = reg_addr | 0x80; //set the MSB to 1 to indicate a read
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    *reg_val = rx_buffer[1];
    return status;
}



uint8_t LIS2DU12_init(SPI_HandleTypeDef *hspi){

    HAL_StatusTypeDef status;

    //Test we can read from sensor
    uint8_t WHO_AM_I_value = 0;
    status = LIS2DU12_read_register(hspi, LIS2DU12_REG_WHO_AM_I, &WHO_AM_I_value);
    HAL_Delay(1);

    //Full scale +-4g
    //25Hz normal mode
    //No samples discarded
    CTRL5_reg.field.FS = 0b01;
    CTRL5_reg.field.BW = 0b00;
    CTRL5_reg.field.ODR = 0b0110;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_CTRL5, CTRL5_reg.raw);
    HAL_Delay(1);

    //FIFO continous mode
    FIFO_CTRL_reg.field.FIFO_MODE = 0b110;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_FIFO_CTRL, FIFO_CTRL_reg.raw);
    HAL_Delay(1);

    //Set FIFO watermark to 50 samples
    //At ODR 25Hz, FIFO will hit watermark every 2 seconds
    FIFO_WTM_reg.field.FTH = 50;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_FIFO_WTM, FIFO_WTM_reg.raw);
    HAL_Delay(1);

    //Enable FIFO threshold interrupt on INT1 pin
    CTRL2_reg.field.INT1_F_FTH = 1;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_CTRL2, CTRL2_reg.raw);
    HAL_Delay(1);



    if (status == HAL_OK){
        return 0;
    }
    else{
        return 1;
    }
}

uint8_t LIS2DU12_read_FIFO(SPI_HandleTypeDef *hspi){
    
    HAL_StatusTypeDef status;

    //Check how many unread samples there are in the FIFO
    status = LIS2DU12_read_register(hspi, LIS2DU12_REG_FIFO_STATUS2, &FIFO_STATUS2_reg.raw);
    printf("Unread FIFO size: %d\r\n", FIFO_STATUS2_reg.field.FSS);
    HAL_Delay(1);

    //Check status of FIFO watermark
    status = LIS2DU12_read_register(hspi, LIS2DU12_REG_FIFO_STATUS1, &FIFO_STATUS1_reg.raw);
    printf("FIFO watermark flag: %d\r\n", FIFO_STATUS1_reg.field.FTH);
    HAL_Delay(1);

    if (status == HAL_OK){
        return 0;
    }
    else{
        return 1;
    }

    return 0;
}
