#include "LIS2DU12.h"
#include "stm32u0xx_hal.h"
#include "stm32u0xx_hal_gpio.h"
#include "stm32u0xx_hal_spi.h"
#include <stdio.h>
#include <string.h>

#define SPI_READ_BIT(reg)  ((reg) | 0x80)
#define SPI_WRITE_BIT(reg)  ((reg) & 0x7F)
#define SPI_TIMEOUT 1000

typedef enum {
    LIS2DU12_REG_CTRL1       = 0x10,
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
        uint8_t WU_Z_EN   : 1;  // bit 0
        uint8_t WU_Y_EN   : 1;  // bit 1
        uint8_t WU_X_EN   : 1;  // bit 2
        uint8_t DRDY_PULSED  : 1;  // bit 3
        uint8_t IF_ADD_INC : 1;  // bit 4
        uint8_t SW_RESET : 1;  // bit 5
        uint8_t SIM: 1;  // bit 6
        uint8_t PP_OD  : 1;  // bit 7
    } field;
} CTRL1_reg;

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

HAL_StatusTypeDef LIS2DU12_read_register(SPI_HandleTypeDef *hspi, LIS2DU12_RegAddr_t reg_addr, uint8_t *return_val){

    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    tx_buffer[0] = reg_addr | 0x80; //set the MSB to 1 to indicate a read
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    *return_val = rx_buffer[1];
    return status;
}



uint8_t LIS2DU12_init(SPI_HandleTypeDef *hspi){

    HAL_StatusTypeDef status;

    //Test we can read from sensor
    uint8_t WHO_AM_I_value = 0;
    status = LIS2DU12_read_register(hspi, LIS2DU12_REG_WHO_AM_I, &WHO_AM_I_value);
    HAL_Delay(1);

    //Enable register address to automatically increment during a bulti byte read
    CTRL1_reg.field.IF_ADD_INC = 1;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_CTRL1, CTRL1_reg.raw);
    HAL_Delay(1);

    //Full scale +-4g
    //25Hz normal mode
    //No samples discarded
    CTRL5_reg.field.FS = 0b01;
    CTRL5_reg.field.BW = 0b00;
    CTRL5_reg.field.ODR = 0b0110;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_CTRL5, CTRL5_reg.raw);
    HAL_Delay(1);

    //FIFO 2x depth mode (8 bit samples, no temperature in FIFO)
    //FIFO continous mode
    FIFO_CTRL_reg.field.FIFO_DEPTH = 1;
    FIFO_CTRL_reg.field.FIFO_MODE = 0b110;
    status = LIS2DU12_write_register(hspi, LIS2DU12_REG_FIFO_CTRL, FIFO_CTRL_reg.raw);
    HAL_Delay(1);

    //Set FIFO watermark to 25 samples.
    //At ODR 25Hz, 2x depth mode, FIFO will hit watermark every 2 seconds
    FIFO_WTM_reg.field.FTH = 25;
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

uint8_t LIS2DU12_read_FIFO(SPI_HandleTypeDef *hspi, uint8_t *FIFO_length, int16_t acceleration_data[256][3]){
    
    HAL_StatusTypeDef status;

    //Check how many unread samples there are in the FIFO
    //Return if no undread samples
    status = LIS2DU12_read_register(hspi, LIS2DU12_REG_FIFO_STATUS2, &FIFO_STATUS2_reg.raw);
    *FIFO_length = FIFO_STATUS2_reg.field.FSS;
    uint8_t num_unread_samples = FIFO_STATUS2_reg.field.FSS; // 3 bytes per sample in 2x depth mode (8-bit XYZ)
    if ((num_unread_samples == 0) || (status != HAL_OK)){
        return 1;
    }

    HAL_Delay(1);

    //Determine how many bytes to read
    static uint8_t rx_buffer[769]; // 256 samples × 3 bytes max
    memset(rx_buffer, 0, 769);
    uint16_t num_bytes_to_read = (uint16_t)num_unread_samples * 6;  //In 2x depth mode each FIFO word is 6 bytes (2 samples in each FIFO word)
    if (num_bytes_to_read > sizeof(rx_buffer)) num_bytes_to_read = sizeof(rx_buffer);
    
    //Read the data
    uint8_t addr = LIS2DU12_REG_FIFO_STATUS2 | 0x80;
    uint8_t status_dummy = 0;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    status = HAL_SPI_TransmitReceive(hspi, &addr, &status_dummy, 1, SPI_TIMEOUT);
    status = HAL_SPI_TransmitReceive(hspi, rx_buffer, rx_buffer, num_bytes_to_read, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    HAL_Delay(1);

    if (status != HAL_OK){
        return 1;
    }

    //Format the raw data into the acceleration data array
    //rx buffer data formatted: x1,y1,z1,x2,y2,z2,x3,y3,z3... etc.
    uint16_t sample_index = 0;
    uint16_t rx_buff_index = 1; //offset by 1 since the first byte is the value of the FIFO_STATUS2 register

    for (uint16_t word = 0; word < num_unread_samples; word++) {
        // Each word = 6 bytes: X_curr, Y_curr, Z_curr, X_prev, Y_prev, Z_prev
        acceleration_data[sample_index][0] = (int16_t)((int32_t)(int8_t)rx_buffer[rx_buff_index++] * 1000 / 32);
        //acceleration_data[sample_index][0] = rx_buffer[rx_buff_index++];
        acceleration_data[sample_index][1] = (int16_t)((int32_t)(int8_t)rx_buffer[rx_buff_index++] * 1000 / 32);
        acceleration_data[sample_index][2] = (int16_t)((int32_t)(int8_t)rx_buffer[rx_buff_index++] * 1000 / 32);
        sample_index++;
        acceleration_data[sample_index][0] = (int16_t)((int32_t)(int8_t)rx_buffer[rx_buff_index++] * 1000 / 32);
        acceleration_data[sample_index][1] = (int16_t)((int32_t)(int8_t)rx_buffer[rx_buff_index++] * 1000 / 32);
        acceleration_data[sample_index][2] = (int16_t)((int32_t)(int8_t)rx_buffer[rx_buff_index++] * 1000 / 32);
        sample_index++;
    }

    return 0;
}
