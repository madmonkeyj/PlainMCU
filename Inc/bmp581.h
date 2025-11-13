/**
  ******************************************************************************
  * @file    bmp581.h
  * @brief   BMP581 barometer driver - Based on Bosch BMP5 reference driver
  ******************************************************************************
  */

#ifndef BMP581_H_
#define BMP581_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* BMP581 I2C Address */
#define BMP581_I2C_ADDR             (0x47 << 1)  // SDO to VDD

/* BMP581 Register Addresses */
#define BMP581_REG_CHIP_ID          0x01
#define BMP581_REG_REV_ID           0x02
#define BMP581_REG_CHIP_STATUS      0x11
#define BMP581_REG_INT_CONFIG       0x14
#define BMP581_REG_INT_SOURCE       0x15
#define BMP581_REG_FIFO_CONFIG      0x16
#define BMP581_REG_FIFO_COUNT       0x17
#define BMP581_REG_FIFO_SEL         0x18
#define BMP581_REG_TEMP_DATA_XLSB   0x1D
#define BMP581_REG_TEMP_DATA_LSB    0x1E
#define BMP581_REG_TEMP_DATA_MSB    0x1F
#define BMP581_REG_PRESS_DATA_XLSB  0x20
#define BMP581_REG_PRESS_DATA_LSB   0x21
#define BMP581_REG_PRESS_DATA_MSB   0x22
#define BMP581_REG_INT_STATUS       0x27
#define BMP581_REG_STATUS           0x28
#define BMP581_REG_FIFO_DATA        0x29
#define BMP581_REG_NVM_ADDR         0x2B
#define BMP581_REG_NVM_DATA_LSB     0x2C
#define BMP581_REG_NVM_DATA_MSB     0x2D
#define BMP581_REG_DSP_CONFIG       0x30
#define BMP581_REG_DSP_IIR          0x31
#define BMP581_REG_OOR_THR_P_LSB    0x32
#define BMP581_REG_OOR_RANGE        0x34
#define BMP581_REG_OOR_CONFIG       0x35
#define BMP581_REG_OSR_CONFIG       0x36
#define BMP581_REG_ODR_CONFIG       0x37
#define BMP581_REG_OSR_EFF          0x38
#define BMP581_REG_CMD              0x7E

/* Expected CHIP_ID value */
#define BMP581_CHIP_ID_VALUE        0x50

/* CMD register commands */
#define BMP581_CMD_NOP              0x00
#define BMP581_CMD_SOFT_RESET       0xB6

/* Delay definitions (in microseconds) */
#define BMP581_DELAY_US_SOFT_RESET  2000    // 2ms (t_soft_res from datasheet)
#define BMP581_DELAY_US_STANDBY     5000    // 5ms (increased for robust mode transitions)
#define BMP581_DELAY_US_POWERUP     50000   // 50ms (increased for robust power-up)
#define BMP581_DELAY_US_MEASURE     40000   // 40ms for first measurement

/* STATUS register bits (0x28) */
#define BMP581_STATUS_NVM_RDY       0x02    // NVM ready (bit 1)
#define BMP581_STATUS_NVM_ERR       0x04    // NVM error (bit 2)

/* INT_STATUS register bits (0x27) */
// --- FIX: Corrected data ready flag to point to the right register and bit ---
#define BMP581_INT_STATUS_DRDY      0x01    // Data ready for temp and/or pressure (bit 0)
#define BMP581_INT_STATUS_POR       0x10    // POR/soft-reset complete (bit 4)

/* CHIP_STATUS register bits (0x11) */
#define BMP581_CHIP_STATUS_POR      0x01    // POR detected
#define BMP581_CHIP_STATUS_BOOT     0x02    // Boot complete

/* ODR (Output Data Rate) values */
#define BMP581_ODR_240_HZ           0x00
#define BMP581_ODR_50_HZ            0x0F    // Target for EKF
#define BMP581_ODR_25_HZ            0x14
// Add other ODR values as needed from datasheet

/* Power mode bits in ODR_CONFIG register */
#define BMP581_MODE_STANDBY         0x00
#define BMP581_MODE_NORMAL          0x01
#define BMP581_MODE_FORCED          0x02
// --- FIX: Corrected mode shift. Power mode is in bits [1:0], so no shift is needed. ---
#define BMP581_MODE_SHIFT           0
#define BMP581_MODE_MASK            0x03

/* Deep standby control */
#define BMP581_DEEP_DISABLE_POS     7
#define BMP581_DEEP_ENABLED         0x00
#define BMP581_DEEP_DISABLED        0x01

/* OSR (Oversampling) values */
#define BMP581_OSR_P_8X             0x03
#define BMP581_OSR_T_2X             0x01
#define BMP581_OSR_T_SHIFT          3

/* Pressure enable */
#define BMP581_PRESS_EN_POS         6

/* IIR Filter coefficients */
#define BMP581_IIR_COEFF_3          0x02

/* Conversion constants (from datasheet) */
#define BMP581_PRESS_SCALE          (1.0f / 64.0f)     // Pa per LSB (2^6)
#define BMP581_TEMP_SCALE           (1.0f / 65536.0f)  // °C per LSB (2^16)

/* Timeout */
#define BMP581_I2C_TIMEOUT          100

/* Data structure for sensor readings */
typedef struct {
    int32_t pressure_raw;       // Raw 24-bit pressure data
    int32_t temperature_raw;    // Raw 24-bit temperature data
    float pressure_pa;          // Pressure in Pascals
    float temperature_c;        // Temperature in Celsius
    float altitude_m;           // Calculated altitude in meters
} BMP581_Data_t;

/* Power mode enumeration */
typedef enum {
    BMP581_POWERMODE_STANDBY = 0,
    BMP581_POWERMODE_NORMAL = 1,
    BMP581_POWERMODE_FORCED = 2,
    BMP581_POWERMODE_DEEP_STANDBY = 4 // This is a virtual mode handled by the driver
} BMP581_PowerMode_t;

/* Function prototypes */
HAL_StatusTypeDef BMP581_Init(void);
HAL_StatusTypeDef BMP581_ReadChipID(uint8_t *chip_id);
HAL_StatusTypeDef BMP581_ReadSensorData(BMP581_Data_t *data);
HAL_StatusTypeDef BMP581_GetPressure(float *pressure_pa);
HAL_StatusTypeDef BMP581_GetTemperature(float *temperature_c);
HAL_StatusTypeDef BMP581_GetAltitude(float *altitude_m, float sea_level_pa);
HAL_StatusTypeDef BMP581_CheckDataReady(bool *ready);
HAL_StatusTypeDef BMP581_WaitForDataReady(uint32_t timeout_ms);
HAL_StatusTypeDef BMP581_GetPowerMode(BMP581_PowerMode_t *powermode);
HAL_StatusTypeDef BMP581_SetPowerMode(BMP581_PowerMode_t powermode);
HAL_StatusTypeDef BMP581_SoftReset(void);
HAL_StatusTypeDef BMP581_ReadRegister(uint8_t reg, uint8_t *value);

/* DMA completion callback */
void BMP581_DMA_Complete_Callback(void);
uint32_t BMP581_GetDMACallbackCount(void);

/* Diagnostic functions */
void BMP581_DumpRegisters(void);
void BMP581_TestDataUpdating(void);
void BMP581_DiagnoseIssue(void);

#endif /* BMP581_H_ */
