#include "LIS2DW12.h"
#include "stm32u0xx_hal.h"
#include "stm32u0xx_hal_gpio.h"
#include "stm32u0xx_hal_spi.h"
#include <stdio.h>
#include <string.h>

#define SPI_READ_BIT(reg)  ((reg) | 0x80)
#define SPI_WRITE_BIT(reg)  ((reg) & 0x7F)
#define SPI_TIMEOUT 1000

#define LIS2DW12_WHO_AM_I_VALUE 0x44

//Filter settle time after the ODR/FS change in configure_sleep, in ms.
//Must span several ODR periods at the sleep ODR (1.6 Hz -> 625 ms per sample).
#define LIS2DW12_SLEEP_SETTLE_MS 2000

typedef enum {
    LIS2DW12_REG_WHO_AM_I       = 0x0F,
    LIS2DW12_REG_CTRL1          = 0x20,
    LIS2DW12_REG_CTRL2          = 0x21,
    LIS2DW12_REG_CTRL3          = 0x22,
    LIS2DW12_REG_CTRL4_INT1_PAD = 0x23,
    LIS2DW12_REG_CTRL5_INT2_PAD = 0x24,
    LIS2DW12_REG_CTRL6          = 0x25,
    LIS2DW12_REG_STATUS         = 0x27,
    LIS2DW12_REG_OUT_X_L        = 0x28,
    LIS2DW12_REG_FIFO_CTRL      = 0x2E,
    LIS2DW12_REG_FIFO_SAMPLES   = 0x2F,
    LIS2DW12_REG_WAKE_UP_THS    = 0x34,
    LIS2DW12_REG_WAKE_UP_DUR    = 0x35,
    LIS2DW12_REG_WAKE_UP_SRC    = 0x38,
    LIS2DW12_REG_ALL_INT_SRC    = 0x3B,
    LIS2DW12_REG_CTRL7          = 0x3F,
} LIS2DW12_RegAddr_t;


// CTRL1 (20h): output data rate, mode and low-power mode selection
static union {
    uint8_t raw;
    struct {
        uint8_t LP_MODE : 2;  // bits 0-1  low-power mode (LP1..LP4)
        uint8_t MODE    : 2;  // bits 2-3  00: low-power, 01: high-performance, 10: single-conv
        uint8_t ODR     : 4;  // bits 4-7  output data rate
    } field;
} CTRL1_reg;

// CTRL2 (21h): interface and reset control
static union {
    uint8_t raw;
    struct {
        uint8_t SIM         : 1;  // bit 0  SPI 3/4-wire
        uint8_t I2C_DISABLE : 1;  // bit 1
        uint8_t IF_ADD_INC  : 1;  // bit 2  auto-increment register address on burst
        uint8_t BDU         : 1;  // bit 3  block data update
        uint8_t CS_PU_DISC  : 1;  // bit 4
        uint8_t reserved    : 1;  // bit 5  must stay 0
        uint8_t SOFT_RESET  : 1;  // bit 6
        uint8_t BOOT        : 1;  // bit 7
    } field;
} CTRL2_reg;

// CTRL3 (22h): interrupt pad electrical config, self-test, single-conversion
static union {
    uint8_t raw;
    struct {
        uint8_t SLP_MODE_1   : 1;  // bit 0
        uint8_t SLP_MODE_SEL : 1;  // bit 1
        uint8_t reserved     : 1;  // bit 2
        uint8_t H_LACTIVE    : 1;  // bit 3  0: interrupts active high, 1: active low
        uint8_t LIR          : 1;  // bit 4  0: pulsed, 1: latched
        uint8_t PP_OD        : 1;  // bit 5  0: push-pull, 1: open-drain
        uint8_t ST           : 2;  // bits 6-7  self-test
    } field;
} CTRL3_reg;

// CTRL4_INT1_PAD_CTRL (23h): routing of functions to the INT1 pad
static union {
    uint8_t raw;
    struct {
        uint8_t INT1_DRDY       : 1;  // bit 0
        uint8_t INT1_FTH        : 1;  // bit 1  FIFO threshold
        uint8_t INT1_DIFF5      : 1;  // bit 2  FIFO full
        uint8_t INT1_TAP        : 1;  // bit 3  double-tap
        uint8_t INT1_FF         : 1;  // bit 4  free-fall
        uint8_t INT1_WU         : 1;  // bit 5  wake-up
        uint8_t INT1_SINGLE_TAP : 1;  // bit 6
        uint8_t INT1_6D         : 1;  // bit 7
    } field;
} CTRL4_INT1_PAD_reg;

// CTRL5_INT2_PAD_CTRL (24h): routing of functions to the INT2 pad
static union {
    uint8_t raw;
    struct {
        uint8_t INT2_DRDY        : 1;  // bit 0
        uint8_t INT2_FTH         : 1;  // bit 1  FIFO threshold
        uint8_t INT2_DIFF5       : 1;  // bit 2  FIFO full
        uint8_t INT2_OVR         : 1;  // bit 3  FIFO overrun
        uint8_t INT2_DRDY_T      : 1;  // bit 4  temperature data-ready
        uint8_t INT2_BOOT        : 1;  // bit 5
        uint8_t INT2_SLEEP_CHG   : 1;  // bit 6  sleep state change
        uint8_t INT2_SLEEP_STATE : 1;  // bit 7  sleep state level
    } field;
} CTRL5_INT2_PAD_reg;

// CTRL6 (25h): bandwidth, full-scale, filter path and low-noise
static union {
    uint8_t raw;
    struct {
        uint8_t reserved  : 2;  // bits 0-1
        uint8_t LOW_NOISE : 1;  // bit 2
        uint8_t FDS       : 1;  // bit 3  0: low-pass path, 1: high-pass path
        uint8_t FS        : 2;  // bits 4-5  00:+-2g 01:+-4g 10:+-8g 11:+-16g
        uint8_t BW_FILT   : 2;  // bits 6-7  digital filter cutoff
    } field;
} CTRL6_reg;

// FIFO_CTRL (2Eh): FIFO mode and watermark threshold (32-slot FIFO)
static union {
    uint8_t raw;
    struct {
        uint8_t FTH   : 5;  // bits 0-4  watermark level (0-31)
        uint8_t FMode : 3;  // bits 5-7  000:bypass 001:FIFO 110:continuous
    } field;
} FIFO_CTRL_reg;

// FIFO_SAMPLES (2Fh): unread sample count and status flags
static union {
    uint8_t raw;
    struct {
        uint8_t Diff     : 6;  // bits 0-5  unread samples (0=empty, 32=full)
        uint8_t FIFO_OVR : 1;  // bit 6
        uint8_t FIFO_FTH : 1;  // bit 7  threshold reached
    } field;
} FIFO_SAMPLES_reg;

// WAKE_UP_THS (34h): tap enable, sleep enable, wake-up threshold
static union {
    uint8_t raw;
    struct {
        uint8_t WK_THS            : 6;  // bits 0-5  1 LSB = 1/64 of FS
        uint8_t SLEEP_ON          : 1;  // bit 6  activity/inactivity enable
        uint8_t SINGLE_DOUBLE_TAP : 1;  // bit 7
    } field;
} WAKE_UP_THS_reg;

// WAKE_UP_DUR (35h): free-fall, wake-up and sleep durations
static union {
    uint8_t raw;
    struct {
        uint8_t SLEEP_DUR  : 4;  // bits 0-3  1 LSB = 512/ODR
        uint8_t STATIONARY : 1;  // bit 4
        uint8_t WAKE_DUR   : 2;  // bits 5-6  1 LSB = 1/ODR
        uint8_t FF_DUR5    : 1;  // bit 7
    } field;
} WAKE_UP_DUR_reg;

// WAKE_UP_SRC (38h): wake-up event source (read-only)
static union {
    uint8_t raw;
    struct {
        uint8_t Z_WU           : 1;  // bit 0
        uint8_t Y_WU           : 1;  // bit 1
        uint8_t X_WU           : 1;  // bit 2
        uint8_t WU_IA          : 1;  // bit 3  wake-up event detected
        uint8_t SLEEP_STATE_IA : 1;  // bit 4
        uint8_t FF_IA          : 1;  // bit 5
        uint8_t reserved       : 2;  // bits 6-7
    } field;
} WAKE_UP_SRC_reg;

// CTRL7 (3Fh): interrupt enable, signal routing and offset/filter options
static union {
    uint8_t raw;
    struct {
        uint8_t LPASS_ON6D        : 1;  // bit 0
        uint8_t HP_REF_MODE       : 1;  // bit 1
        uint8_t USR_OFF_W         : 1;  // bit 2
        uint8_t USR_OFF_ON_WU     : 1;  // bit 3
        uint8_t USR_OFF_ON_OUT    : 1;  // bit 4
        uint8_t INTERRUPTS_ENABLE : 1;  // bit 5  global enable for wake-up/tap/FF/6D/activity
        uint8_t INT2_ON_INT1      : 1;  // bit 6
        uint8_t DRDY_PULSED       : 1;  // bit 7
    } field;
} CTRL7_reg;


HAL_StatusTypeDef LIS2DW12_write_register(SPI_HandleTypeDef *hspi, LIS2DW12_RegAddr_t reg_addr, uint8_t reg_val){

    uint8_t tx_buffer[2] = {0};
    tx_buffer[0] = reg_addr & 0x7F; //clear the MSB to indicate a write
    tx_buffer[1] = reg_val;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, tx_buffer, 2, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    return status;
}

HAL_StatusTypeDef LIS2DW12_read_register(SPI_HandleTypeDef *hspi, LIS2DW12_RegAddr_t reg_addr, uint8_t *return_val){

    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};

    tx_buffer[0] = reg_addr | 0x80; //set the MSB to 1 to indicate a read
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    *return_val = rx_buffer[1];
    return status;
}



uint8_t LIS2DW12_init(SPI_HandleTypeDef *hspi){

    HAL_StatusTypeDef status;

    //Test we can read from sensor (WHO_AM_I should read 0x44)
    uint8_t WHO_AM_I_value = 0;
    status = LIS2DW12_read_register(hspi, LIS2DW12_REG_WHO_AM_I, &WHO_AM_I_value);
    __NOP();
    if (WHO_AM_I_value != LIS2DW12_WHO_AM_I_VALUE){
        __NOP();
        return 1;
    }
    HAL_Delay(1);

    //Soft reset before configuring anything. Waking from Shutdown resets the MCU but
    //NOT the sensor: the shadow registers above are statics that come back zeroed while
    //the device still holds whatever configure_sleep left in it. Registers that
    //configure_sleep writes but init does not (CTRL3, CTRL4, CTRL7, WAKE_UP_THS,
    //WAKE_UP_DUR) would otherwise stay stale, so the shadows would lie about the device
    //and a later read-modify-write would clear bits it never meant to touch. Resetting
    //here makes "shadow == device" true by construction on every boot, cold or woken.
    CTRL2_reg.raw = 0;
    CTRL2_reg.field.SOFT_RESET = 1;
    status = LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL2, CTRL2_reg.raw);
    if (status != HAL_OK){
        return 1;
    }

    //SOFT_RESET self-clears once the reset completes (a few microseconds). Poll rather
    //than guess, but bound it so a dead device cannot hang the boot.
    uint32_t reset_timeout = HAL_GetTick() + 10;
    do {
        if (LIS2DW12_read_register(hspi, LIS2DW12_REG_CTRL2, &CTRL2_reg.raw) != HAL_OK){
            return 1;
        }
    } while (CTRL2_reg.field.SOFT_RESET && (HAL_GetTick() < reset_timeout));

    if (CTRL2_reg.field.SOFT_RESET){
        return 1;  //reset never completed - treat as a dead sensor
    }

    //Device is now at its power-on defaults, so bring every shadow back in step with it.
    //All of these default to 0x00; CTRL2 defaults to 0x04 (IF_ADD_INC set) but is
    //rewritten from scratch immediately below, so zeroing it here is safe.
    CTRL1_reg.raw = 0;
    CTRL2_reg.raw = 0;
    CTRL3_reg.raw = 0;
    CTRL4_INT1_PAD_reg.raw = 0;
    CTRL5_INT2_PAD_reg.raw = 0;
    CTRL6_reg.raw = 0;
    CTRL7_reg.raw = 0;
    FIFO_CTRL_reg.raw = 0;
    WAKE_UP_THS_reg.raw = 0;
    WAKE_UP_DUR_reg.raw = 0;

    //Enable register address to automatically increment during a multi byte read.
    //Also disable I2C: with both interfaces live the device picks its mode from CS,
    //so the moment CS goes high (as it does for the whole of Shutdown) it switches to
    //I2C and reinterprets the pads - SDO becomes the SA0 address input rather than a
    //tri-stated SPI output. Pinning it to SPI keeps the pad behaviour defined while asleep.
    CTRL2_reg.field.IF_ADD_INC = 1;
    CTRL2_reg.field.I2C_DISABLE = 1;
    status = LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL2, CTRL2_reg.raw);
    HAL_Delay(1);

    //12.5 Hz, low-power mode 2 (14-bit).
    //High-performance mode draws ~90uA regardless of ODR, which dwarfs everything the
    //MCU saves by sleeping between beats; low-power mode 2 at 12.5 Hz is ~1.6uA. The
    //cost is noise density rising from 110 to 300 ug/sqrt(Hz), which does not matter
    //here because the movement score is built from sample-to-sample deltas averaged
    //over 255 pairs, so uncorrelated noise averages out.
    //12.5 Hz also makes the 32-slot FIFO span 2.56s, comfortably longer than the 2.24s
    //worst-case gap between reads at movement score 0, so no samples are dropped.
    CTRL1_reg.field.ODR = 0b0010;  //12.5 Hz
    CTRL1_reg.field.MODE = 0b00;   //low-power
    CTRL1_reg.field.LP_MODE = 0b01;//low-power mode 2 (14-bit)
    status = LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL1, CTRL1_reg.raw);
    HAL_Delay(1);

    //Full scale +-4g, ODR/2 bandwidth, low-pass path
    CTRL6_reg.field.FS = 0b01;
    CTRL6_reg.field.BW_FILT = 0b00;
    CTRL6_reg.field.FDS = 0;
    CTRL6_reg.field.LOW_NOISE = 0;
    status = LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL6, CTRL6_reg.raw);
    HAL_Delay(1);

    //FIFO continuous mode with a 25-sample watermark.
    //The 32-slot FIFO stores one 14-bit XYZ set per sample. At 12.5 Hz the
    //watermark is reached every ~2 seconds and the FIFO only fills at 2.56s, so
    //there is margin against the slowest read cadence. Mode and threshold share
    //FIFO_CTRL.
    FIFO_CTRL_reg.field.FMode = 0b110;  //continuous
    FIFO_CTRL_reg.field.FTH = 25;
    status = LIS2DW12_write_register(hspi, LIS2DW12_REG_FIFO_CTRL, FIFO_CTRL_reg.raw);
    HAL_Delay(1);

    //Route the FIFO threshold interrupt to the INT2 pad (accelerometer INT2 -> PA2)
    CTRL5_INT2_PAD_reg.field.INT2_FTH = 1;
    status = LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL5_INT2_PAD, CTRL5_INT2_PAD_reg.raw);
    HAL_Delay(1);



    if (status == HAL_OK){
        return 0;
    }
    else{
        return 1;
    }
}

uint8_t LIS2DW12_read_FIFO(SPI_HandleTypeDef *hspi, uint8_t *FIFO_length, int8_t acceleration_data[256][3]){

    HAL_StatusTypeDef status;

    //Check how many unread samples there are in the FIFO.
    //Return if there are no unread samples.
    status = LIS2DW12_read_register(hspi, LIS2DW12_REG_FIFO_SAMPLES, &FIFO_SAMPLES_reg.raw);
    *FIFO_length = FIFO_SAMPLES_reg.field.Diff;
    uint8_t num_unread_samples = FIFO_SAMPLES_reg.field.Diff;
    if ((num_unread_samples == 0) || (status != HAL_OK)){
        return 1;
    }

    HAL_Delay(1);

    //Determine how many bytes to read. The FIFO holds at most 32 samples and
    //each sample is a 6-byte XYZ set (16-bit left-justified per axis).
    static uint8_t rx_buffer[32 * 6];
    memset(rx_buffer, 0, sizeof(rx_buffer));
    if (num_unread_samples > 32) num_unread_samples = 32;
    uint16_t num_bytes_to_read = (uint16_t)num_unread_samples * 6;

    //Burst read from OUT_X_L. With IF_ADD_INC set the device walks 0x28..0x2D and
    //rolls back to 0x28, pulling the next FIFO sample set each time around.
    uint8_t addr = LIS2DW12_REG_OUT_X_L | 0x80;
    uint8_t addr_dummy = 0;
    static uint8_t tx_buffer[32 * 6];
    memset(tx_buffer, 0, num_bytes_to_read);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
    status = HAL_SPI_TransmitReceive(hspi, &addr, &addr_dummy, 1, SPI_TIMEOUT);
    status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, num_bytes_to_read, SPI_TIMEOUT);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
    HAL_Delay(1);

    if (status != HAL_OK){
        return 1;
    }

    //Shift existing samples toward index 0 to make room, discarding the oldest ones that age out of the window,
    //so newest sample always ends up at index 255 and callers never need to track a cursor.
    uint16_t new_sample_count = num_unread_samples;
    if (new_sample_count < 256) {
        memmove(&acceleration_data[0], &acceleration_data[new_sample_count], (256 - new_sample_count) * sizeof(acceleration_data[0]));
    }

    uint16_t sample_index = 256 - new_sample_count;
    uint16_t rx_buff_index = 0;

    for (uint16_t sample = 0; sample < num_unread_samples; sample++) {
        // Each sample = 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H (14-bit left-justified).
        // Keep the high byte of each axis to preserve the int8_t interface; this is
        // the top 8 bits of the signed value, which is all the movement score needs.
        acceleration_data[sample_index][0] = (int8_t)rx_buffer[rx_buff_index + 1]; //X_H
        acceleration_data[sample_index][1] = (int8_t)rx_buffer[rx_buff_index + 3]; //Y_H
        acceleration_data[sample_index][2] = (int8_t)rx_buffer[rx_buff_index + 5]; //Z_H
        rx_buff_index += 6;
        sample_index++;
    }

    return 0;
}


uint8_t LIS2DW12_configure_sleep(SPI_HandleTypeDef *hspi){

    uint8_t error = 0;

    //Disable FIFO (bypass mode): nothing needs to be collected while sleeping,
    //only a wake-up event on INT1 is needed.
    FIFO_CTRL_reg.field.FMode = 0b000;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_FIFO_CTRL, FIFO_CTRL_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //Unroute everything from the INT2 pad. Only INT1 is a wake source, so anything
    //still routed here can only assert into a sleeping MCU and drive the pad against
    //the board for the whole sleep without ever waking us to show it.
    CTRL5_INT2_PAD_reg.raw = 0;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL5_INT2_PAD, CTRL5_INT2_PAD_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //1.6 Hz, low-power mode 1 (12-bit) - lowest current draw (~0.38uA typ)
    CTRL1_reg.field.ODR = 0b0001;  //1.6 Hz in low-power
    CTRL1_reg.field.MODE = 0b00;   //low-power mode
    CTRL1_reg.field.LP_MODE = 0b00;//low-power mode 1
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL1, CTRL1_reg.raw) != HAL_OK);

    //Full scale +-2g, ODR/2 bandwidth, low-pass path
    CTRL6_reg.field.FS = 0b00;
    CTRL6_reg.field.BW_FILT = 0b00;
    CTRL6_reg.field.FDS = 0;
    CTRL6_reg.field.LOW_NOISE = 0;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL6, CTRL6_reg.raw) != HAL_OK);

    //Changing ODR/FS resets the internal filter the wake-up engine reads from.
    //Give it time to settle before arming the threshold, otherwise the settling transient
    //itself reads as a false wake-up event on a stationary device.
    //
    //This has to be counted in ODR periods, and 1.6 Hz is a 625 ms period - the previous
    //600 ms here was less than a single sample, so the transient was still in flight when
    //the threshold was armed. INT1 then came up immediately, and because WKUP3 is
    //level-triggered the MCU woke straight back out of Shutdown. 2000 ms is 3.2 periods.
    //The caller still confirms INT1 is actually low before arming the wake pin, so this
    //delay no longer has to be exactly right - it just has to get most of the way there.
    HAL_Delay(LIS2DW12_SLEEP_SETTLE_MS);

    //Wake-up threshold. 1 LSB = FS/(2^6) = 2000mg/64 ~= 31mg.
    //WK_THS = 4 -> ~125mg. Tune to taste. No sleep/inactivity engine needed:
    //the wake-up event alone is routed to INT1.
    WAKE_UP_THS_reg.field.WK_THS = 4;
    WAKE_UP_THS_reg.field.SLEEP_ON = 0;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_WAKE_UP_THS, WAKE_UP_THS_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //Wake-up duration: minimum debounce (1 ODR_time)
    WAKE_UP_DUR_reg.field.WAKE_DUR = 0b00;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_WAKE_UP_DUR, WAKE_UP_DUR_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //Interrupt pad electrical config: active high, push-pull, pulsed.
    //(Defaults, but set explicitly so the wake pin drives high on an event.)
    CTRL3_reg.field.H_LACTIVE = 0;
    CTRL3_reg.field.PP_OD = 0;
    CTRL3_reg.field.LIR = 0;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL3, CTRL3_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //Route wake-up recognition to the INT1 pad (accelerometer INT1 -> PA1 wake pin).
    //On the LIS2DW12 the wake-up event can only be routed to INT1.
    CTRL4_INT1_PAD_reg.field.INT1_WU = 1;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL4_INT1_PAD, CTRL4_INT1_PAD_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //Globally enable the interrupt-generating functions (required for wake-up).
    CTRL7_reg.field.INTERRUPTS_ENABLE = 1;
    error |= (LIS2DW12_write_register(hspi, LIS2DW12_REG_CTRL7, CTRL7_reg.raw) != HAL_OK);
    HAL_Delay(1);

    //Arming the engine can itself latch an event from the last of the settling
    //transient. Drop anything pending so the caller starts from a clean pad.
    error |= LIS2DW12_clear_interrupt_sources(hspi);

    return error;
}


//Reset every interrupt flag currently routed to the INT pads. Per the datasheet
//(section 8.31) reading ALL_INT_SRC clears them all simultaneously; WAKE_UP_SRC is
//read afterwards so the wake-up detail bits are available for debugging and so the
//two shadows agree with the device.
//Returns 0 on success.
uint8_t LIS2DW12_clear_interrupt_sources(SPI_HandleTypeDef *hspi){

    uint8_t all_int_src = 0;
    uint8_t error = 0;

    error |= (LIS2DW12_read_register(hspi, LIS2DW12_REG_ALL_INT_SRC, &all_int_src) != HAL_OK);
    error |= (LIS2DW12_read_register(hspi, LIS2DW12_REG_WAKE_UP_SRC, &WAKE_UP_SRC_reg.raw) != HAL_OK);

    return error;
}
